#include "ObjectProjection.hpp"

void ObjectProjection::printPoints2f(const std::vector<cv::Point2f> &points)
{
    int contador = 0;
    for (const auto &point : points)
        std::cout << ++contador << point << std::endl;
}

ObjectProjection::ObjectProjection(const std::vector<cv::Point3f> &vertices,
                                   const std::vector<cv::Point3f> &normals,
                                   const std::vector<cv::Point2f> &texCoords,
                                   const std::vector<Face> &faces,
                                   float maxDistancePercentage,
                                   double lightSpeed,
                                   double lightRadius)
    : vertices(vertices), normals(normals),
      texCoords(texCoords), faces(faces),
      maxDistancePercentage(maxDistancePercentage),
      lightRadius(lightRadius), lightSpeed(lightSpeed)
{
    startTime = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency(); // Initialize start time
}

void ObjectProjection::drawObject(cv::Mat &image, cv::Vec3d rvec, cv::Vec3d tvec,
                                  const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                  cv::Vec3d additionalRotation,
                                  const AnimationConfig &animationConfig)
{
    // Convert rotation vector to rotation matrix
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    cv::Mat addRotXR = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                       0, cos(additionalRotation[0]), -sin(additionalRotation[0]),
                       0, sin(additionalRotation[0]), cos(additionalRotation[0]));
    cv::Mat addRotYR = (cv::Mat_<double>(3, 3) << cos(additionalRotation[1]), 0, sin(additionalRotation[1]),
                       0, 1, 0,
                       -sin(additionalRotation[1]), 0, cos(additionalRotation[1]));
    //en su propio eje
    cv::Mat addRotZR = (cv::Mat_<double>(3, 3) << cos(additionalRotation[2]), -sin(additionalRotation[2]), 0,
                       sin(additionalRotation[2]), cos(additionalRotation[2]), 0,
                       0, 0, 1);


    cv::Mat addRotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(additionalRotation[0]), -sin(additionalRotation[0]), 0, sin(additionalRotation[0]), cos(additionalRotation[0]));
    cv::Mat addRotY = (cv::Mat_<double>(3, 3) << cos(additionalRotation[1]), -sin(additionalRotation[1]), 0, sin(additionalRotation[1]), cos(additionalRotation[1]), 0, 0, 0, 1);
    cv::Mat addRotZ = (cv::Mat_<double>(3, 3) << cos(additionalRotation[2]), 0, sin(additionalRotation[2]), 0, 1, 0, -sin(additionalRotation[2]), 0, cos(additionalRotation[2]));

    if (animationConfig.spin == true)
    {
        rmat = rmat * addRotX * addRotY * addRotZ; // en su propio eje
    }

    if (animationConfig.rotate == true)
    {
        rmat = rmat * addRotXR * addRotYR * addRotZR; // en su propio eje
    }



    cv::Rodrigues(rmat, rvec);

    double currentTime = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
    double time = currentTime - startTime;

    // Oscilación vertical en el eje y usando una función seno
    double oscillationAmplitude = 0.5; // Amplitud de la oscilación en el eje y
    double oscillationFrequency = 1.0; // Frecuencia de la oscilación
    double angle = time * lightSpeed;
    double yOffset = oscillationAmplitude * sin(oscillationFrequency * angle);                    // Oscilación en y
    cv::Point3f lightSourcePosition(lightRadius * cos(angle), yOffset, lightRadius * sin(angle)); // Gira en el plano xz y oscila en y

    // Aplicar la rotación a la posición de la luz
    tvec[0] += animationConfig.xTranslation;
    tvec[1] += animationConfig.yTranslation;
    tvec[2] += animationConfig.scaleObject;
    cv::Mat lightMat = rmat.inv();
    cv::Point3f lightPos = cv::Point3f(
        lightMat.at<double>(0, 0) * lightSourcePosition.x + lightMat.at<double>(0, 1) * lightSourcePosition.y + lightMat.at<double>(0, 2) * lightSourcePosition.z,
        lightMat.at<double>(1, 0) * lightSourcePosition.x + lightMat.at<double>(1, 1) * lightSourcePosition.y + lightMat.at<double>(1, 2) * lightSourcePosition.z,
        lightMat.at<double>(2, 0) * lightSourcePosition.x + lightMat.at<double>(2, 1) * lightSourcePosition.y + lightMat.at<double>(2, 2) * lightSourcePosition.z);

    // Project light position onto the image plane
    std::vector<cv::Point2f> lightSourceImgPts;
    cv::projectPoints(std::vector<cv::Point3f>{lightPos}, rvec, tvec, cameraMatrix, distCoeffs, lightSourceImgPts);

    // Project points
    std::vector<cv::Point2f> imgpts(vertices.size());
    cv::projectPoints(vertices, rvec, tvec, cameraMatrix, distCoeffs, imgpts);

    // Calculate illumination intensity for a triangle
    auto calculateIllumination = [&](const cv::Point3f &p0, const cv::Point3f &p1, const cv::Point3f &p2, const cv::Point3f &lightPos)
    {
        // Calculate the normal of the triangle
        cv::Point3f edge1 = p1 - p0;
        cv::Point3f edge2 = p2 - p0;
        cv::Point3f normal = edge1.cross(edge2);
        double normLength = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        normal /= normLength; // Normalize the normal

        // Calculate the light direction vector from the light position
        cv::Point3f centroid = (p0 + p1 + p2) / 3.0;
        cv::Point3f lightDir = lightPos - centroid;
        double lightDirLength = std::sqrt(lightDir.x * lightDir.x + lightDir.y * lightDir.y + lightDir.z * lightDir.z);
        lightDir /= lightDirLength; // Normalize the light direction

        // Calculate the dot product with the light direction
        double dotProduct = normal.dot(lightDir);

        // Clamp the dot product to be between 0 and 1
        double intensity = std::max(0.0, std::min(1.0, dotProduct));

        return intensity;
    };
    struct FaceInfo
    {
        std::vector<int> vertices;
        double depth; // Depth based on the 2D projection
    };

    // Calcular la profundidad de una cara basada en la proyección 2D
    auto calculateFaceDepth = [&](const std::vector<int> &vertexIndices)
    {
        // Calcular el centroide en el espacio 3D
        cv::Point3f centroid(0.0, 0.0, 0.0);
        for (int index : vertexIndices)
        {
            centroid += vertices[index - 1]; // Sumar las coordenadas de los vértices
        }
        centroid /= static_cast<double>(vertexIndices.size()); // Promediar la posición

        // Convertir el centroide al espacio de la cámara y proyectar al plano 2D
        std::vector<cv::Point2f> imgpts;
        cv::projectPoints(std::vector<cv::Point3f>{centroid}, rvec, tvec, cameraMatrix, distCoeffs, imgpts);

        // Depth is calculated from the centroid position in the image plane
        return imgpts[0].y; // Depth in the image plane (higher y means further from the camera)
    };

    std::vector<FaceInfo> FacesInfos;

    for (const auto &face : faces)
    {
        FaceInfo faceInfo;
        faceInfo.vertices = face.vertices;
        faceInfo.depth = calculateFaceDepth(face.vertices);
        FacesInfos.push_back(faceInfo);
    }

    // Sort base faces and body faces by depth
    std::sort(FacesInfos.begin(), FacesInfos.end(), [](const FaceInfo &a, const FaceInfo &b)
              {
                  return a.depth > b.depth; // Farther faces drawn first
              });

    // Draw the light source on the image
    for (const auto &pt : imgpts)
    {
        cv::circle(image, pt, 2, cv::Scalar(0, 255, 0), -1);
    }
    for (const auto &pt : lightSourceImgPts)
    {
        cv::circle(image, pt, 10, cv::Scalar(255, 255, 255), -1);
    }
    // Draw body faces next
    for (const auto &faceInfo : FacesInfos)
    {
        std::vector<cv::Point> points;
        for (const auto &vertex : faceInfo.vertices)
        {
            points.push_back(imgpts[vertex - 1]);
        }
        double intensity = calculateIllumination(vertices[faceInfo.vertices[0] - 1], vertices[faceInfo.vertices[1] - 1], vertices[faceInfo.vertices[2] - 1], lightPos);
        cv::Scalar fillColor = animationConfig.baseColor * intensity;
        cv::polylines(image, points, true, fillColor, 1);
        cv::fillConvexPoly(image, points, fillColor, cv::LINE_AA);
    }
}