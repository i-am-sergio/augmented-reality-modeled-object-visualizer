#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "Load3DModel.hpp"
#include "Delaunay.hpp"
#include "Delaunay3D.hpp"
#include <vector>
#include <cmath>
#include <unordered_map>
#include <functional>

struct AnimationConfig
{
    bool spin;
    bool iluminate;
    bool rotate;

    double step;

    double xTranslation;
    double yTranslation;
    double scaleObject;

    double rotationSpeedZ;
    double scaleStep;
};


struct CaraNew
{
    std::vector<int> indices; // Lista de índices de los vértices
};
namespace std
{
    template <typename T>
    struct hash<delaunay3D::Point<T>>
    {
        std::size_t operator()(const delaunay3D::Point<T> &p) const
        {
            auto h1 = std::hash<T>{}(p.x);
            auto h2 = std::hash<T>{}(p.y);
            auto h3 = std::hash<T>{}(p.z);
            return h1 ^ (h2 << 1) ^ (h3 << 2); // Combine hash values
        }
    };
}
using namespace cv;
using namespace std;
using namespace delaunay;
using namespace delaunay3D;

double euclideanDistance(const cv::Point3f &p1, const cv::Point3f &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double distance(const cv::Point3f &p1, const cv::Point3f &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}
struct Cara
{
    std::vector<int> indices;
};
struct Point3fCompare
{
    bool operator()(const cv::Point3f &lhs, const cv::Point3f &rhs) const
    {
        if (lhs.x != rhs.x)
            return lhs.x < rhs.x;
        if (lhs.y != rhs.y)
            return lhs.y < rhs.y;
        return lhs.z < rhs.z;
    }
};
void createCaras(const std::vector<delaunay3D::Point<float>> &vertices,
                 const delaunay3D::Delaunay3D<float> &delaunay,
                 std::vector<Cara> &caras)
{
    // Mapea los puntos a sus índices
    std::unordered_map<delaunay3D::Point<float>, int> pointIndexMap;
    for (int i = 0; i < vertices.size(); ++i)
    {
        pointIndexMap[vertices[i]] = i;
    }

    // Llena la lista de Cara con los índices de los vértices de los triángulos
    for (const auto &triangle : delaunay.triangles)
    {
        Cara cara;
        cara.indices.push_back(pointIndexMap[triangle.p0]);
        cara.indices.push_back(pointIndexMap[triangle.p1]);
        cara.indices.push_back(pointIndexMap[triangle.p2]);
        caras.push_back(cara);
    }
}

class ObjectProjection
{
private:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texCoords;
    std::vector<Face> faces;
    std::vector<CaraNew> caras;
    float maxDistancePercentage;
    double lightSpeed;  // Speed of light movement in radians per second
    double lightRadius; // Radius of light orbit
    double startTime;   // Time variable to keep track of the animation

public:
    ObjectProjection(const std::vector<cv::Point3f> &vertices, const std::vector<cv::Point3f> &normals, const std::vector<cv::Point2f> &texCoords, const std::vector<Face> &faces, float maxDistancePercentage, double lightSpeed = 1.0, double lightRadius = 0.4)
        : vertices(vertices), normals(normals), texCoords(texCoords), faces(faces), maxDistancePercentage(maxDistancePercentage), lightRadius(lightRadius), lightSpeed(lightSpeed)
    {
        startTime = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency(); // Initialize start time
    }
    ObjectProjection(const std::vector<cv::Point3f> &vertices, const std::vector<cv::Point3f> &normals, const std::vector<cv::Point2f> &texCoords, const std::vector<CaraNew> &carass, float maxDistancePercentage, double lightSpeed = 1.0, double lightRadius = 0.4)
        : vertices(vertices), normals(normals), texCoords(texCoords), caras(carass), maxDistancePercentage(maxDistancePercentage), lightRadius(lightRadius), lightSpeed(lightSpeed)
    {
        startTime = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency(); // Initialize start time
    }

    void drawObject(cv::Mat &frame, const cv::Vec3d &rvec, const cv::Vec3d &tvec, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, double movement = 0.0)
    {
        cv::Vec3d adjusted_tvec = tvec + cv::Vec3d(0, 0, movement);

        std::vector<cv::Point2f> imgpts;
        cv::projectPoints(vertices, rvec, adjusted_tvec, cameraMatrix, distCoeffs, imgpts);

        for (const auto &pt : imgpts)
        {
            cv::circle(frame, pt, 3, cv::Scalar(22, 21, 250), -1);
        }

        for (const auto &face : faces)
        {
            std::vector<cv::Point> points;
            for (const auto &vertex : face.vertices)
            {
                points.push_back(imgpts[vertex - 1]);
            }
            cv::fillConvexPoly(frame, points, cv::Scalar(255, 179, 153));
        }
    }
    void drawObject(cv::Mat &image, cv::Vec3d rvec, cv::Vec3d tvec, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, cv::Vec3d additionalRotation = cv::Vec3d(0, 0, 0), const AnimationConfig &animationConfig = AnimationConfig())
    {
        // Convert rotation vector to rotation matrix
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        cv::Mat addRotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(additionalRotation[0]), -sin(additionalRotation[0]), 0, sin(additionalRotation[0]), cos(additionalRotation[0]));
        cv::Mat addRotY = (cv::Mat_<double>(3, 3) << cos(additionalRotation[1]), 0, sin(additionalRotation[1]), 0, 1, 0, -sin(additionalRotation[1]), 0, cos(additionalRotation[1]));
        cv::Mat addRotZ = (cv::Mat_<double>(3, 3) << cos(additionalRotation[2]), -sin(additionalRotation[2]), 0, sin(additionalRotation[2]), cos(additionalRotation[2]), 0, 0, 0, 1);
        rmat = rmat * addRotX * addRotY * addRotZ;

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

// Struct to hold information about each face
struct FaceInfo {
    std::vector<int> vertices;
    double depth; // Depth at the centroid of the face
};

// Calculate the depth of a face as the depth of its centroid
auto calculateFaceDepth = [&](const std::vector<int>& vertexIndices) {
    cv::Point3f centroid(0.0, 0.0, 0.0);
    for (int index : vertexIndices) {
        centroid += vertices[index - 1]; // Sum of vertex coordinates
    }
    // Convert vertexIndices.size() to double to avoid ambiguity
    centroid /= static_cast<double>(vertexIndices.size()); // Average position
    return centroid.z; // Depth is the z coordinate of the centroid
};

// Create a list of faces with their depth information
std::vector<FaceInfo> faceInfos;
for (const auto& face : faces) {
    FaceInfo faceInfo;
    faceInfo.vertices = face.vertices;
    faceInfo.depth = calculateFaceDepth(face.vertices);
    faceInfos.push_back(faceInfo);
}

// Sort faces by depth (from farthest to closest)
std::sort(faceInfos.begin(), faceInfos.end(), [](const FaceInfo& a, const FaceInfo& b) {
    return a.depth > b.depth;
});

// Draw the light source on the image
for (const auto &pt : imgpts) {
    cv::circle(image, pt, 2, cv::Scalar(0, 255, 0), -1);
}
for (const auto &pt : lightSourceImgPts) {
    cv::circle(image, pt, 10, cv::Scalar(255, 255, 255), -1);
}

// Draw faces in order of depth
for (const auto &faceInfo : faceInfos) {
    std::vector<cv::Point> points;
    for (const auto &vertex : faceInfo.vertices) {
        points.push_back(imgpts[vertex - 1]);
    }
    double intensity = calculateIllumination(vertices[faceInfo.vertices[0] - 1], vertices[faceInfo.vertices[1] - 1], vertices[faceInfo.vertices[2] - 1], lightPos);
    cv::Scalar fillColor = cv::Scalar(0, 255, 0) * intensity;
    cv::polylines(image, points, true, fillColor, 1);
    cv::fillConvexPoly(image, points, fillColor, cv::LINE_AA);
}

        /*for (const auto &cara : caras)
        {
            std::vector<cv::Point> points;
            for (const auto &index : cara.indices)
            {
                points.push_back(imgpts[index]);
            }
            double intensity = calculateIllumination(vertices[cara.indices[0]], vertices[cara.indices[1]], vertices[cara.indices[2]], lightPos);
            cv::Scalar fillColor = cv::Scalar(0, 255, 0) * intensity;
            cv::polylines(image, points, true, fillColor, 1);
            cv::fillConvexPoly(image, points, fillColor, cv::LINE_AA);
        }*/
    }

    /*void drawObject(cv::Mat &image, cv::Vec3d rvec, cv::Vec3d tvec, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, cv::Vec3d additionalRotation = cv::Vec3d(0, 0, 0))
    {
        // Convert rotation vector to rotation matrix
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        cv::Mat addRotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(additionalRotation[0]), -sin(additionalRotation[0]), 0, sin(additionalRotation[0]), cos(additionalRotation[0]));
        cv::Mat addRotY = (cv::Mat_<double>(3, 3) << cos(additionalRotation[1]), 0, sin(additionalRotation[1]), 0, 1, 0, -sin(additionalRotation[1]), 0, cos(additionalRotation[1]));
        // auto lightpos = rmat; // Extract pos before rotation changes
        rmat = rmat * addRotX * addRotY;
        // Calculate light position in object space
        cv::Mat lightMat = rmat.inv();              // Get the inverse of the rotation matrix
        cv::Point3f lightSourcePosition(0, 0, 0.4); // Light source position in object space
        cv::Point3f lightPos = cv::Point3f(lightMat.at<double>(0, 0) * lightSourcePosition.x + lightMat.at<double>(0, 1) * lightSourcePosition.y + lightMat.at<double>(0, 2) * lightSourcePosition.z,
                                           lightMat.at<double>(1, 0) * lightSourcePosition.x + lightMat.at<double>(1, 1) * lightSourcePosition.y + lightMat.at<double>(1, 2) * lightSourcePosition.z,
                                           lightMat.at<double>(2, 0) * lightSourcePosition.x + lightMat.at<double>(2, 1) * lightSourcePosition.y + lightMat.at<double>(2, 2) * lightSourcePosition.z);
        // Project light position onto the image plane
        std::vector<cv::Point2f> lightSourceImgPts;
        cv::projectPoints(std::vector<cv::Point3f>{lightPos}, rvec, tvec, cameraMatrix, distCoeffs, lightSourceImgPts);

        std::vector<cv::Point2f> imgpts(vertices.size());
        std::vector<cv::Point3f> visibleVertices;
        std::vector<cv::Point3f> notVisibleVertices;

        // Project points
        cv::projectPoints(vertices, rvec, tvec, cameraMatrix, distCoeffs, imgpts);

        // Find the Z-coordinate range for coloring
        double minZ = std::numeric_limits<double>::max();
        double maxZ = std::numeric_limits<double>::lowest();

        for (const auto &vertex : vertices)
        {
            if (vertex.z < minZ)
                minZ = vertex.z;
            if (vertex.z > maxZ)
                maxZ = vertex.z;
        }
        double midpointZ = (minZ + maxZ) / 2.0;
        // Draw points and classify them
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            const auto &pt = imgpts[i];
            const auto &vertex = vertices[i];
            cv::Scalar color;
            if (vertex.z >= midpointZ)
            {
                color = cv::Scalar(0, 255, 0); // Green for points above the midpoint
                visibleVertices.push_back(vertex);
            }
            else
            {
                color = cv::Scalar(255, 0, 255); // Purple for points below the midpoint
                notVisibleVertices.push_back(vertex);
            }
            cv::circle(image, pt, 2, color, -1);
        }

        std::vector<delaunay3D::Point<float>> verticesNew;
        for (const auto &pt : vertices)
            verticesNew.emplace_back(delaunay3D::Point<float>(pt.x, pt.y, pt.z));

        std::vector<delaunay3D::Point<float>> verticesNewVisible;
        for (const auto &pt : visibleVertices)
            verticesNewVisible.emplace_back(delaunay3D::Point<float>(pt.x, pt.y, pt.z));

        std::vector<delaunay3D::Point<float>> verticesNewNotVisible;
        for (const auto &pt : notVisibleVertices)
            verticesNewNotVisible.emplace_back(delaunay3D::Point<float>(pt.x, pt.y, pt.z));

        auto delaunay = triangulatesXY(verticesNewVisible);
        auto delaunayNotVisible = triangulatesXY(verticesNewNotVisible);

        std::vector<Cara> caras, carasNotVisible, carasVisibleExtremes, carasNotVisibleExtremes;
        createCaras(verticesNew, delaunay, caras);
        createCaras(verticesNew, delaunayNotVisible, carasNotVisible);
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

        // Draw the light source on the image
        for (const auto &pt : lightSourceImgPts)
        {
            cv::circle(image, pt, 10, cv::Scalar(255, 255, 255), -1);
        }

        // Draw the triangles for not visible vertices
        for (const auto &cara : carasNotVisible)
        {
            std::vector<cv::Point> points;
            for (const auto &index : cara.indices)
            {
                points.push_back(imgpts[index]);
            }
            double intensity = calculateIllumination(vertices[cara.indices[0]], vertices[cara.indices[1]], vertices[cara.indices[2]], lightPos);
            cv::Scalar fillColor = cv::Scalar(255, 0, 255) * intensity;
            cv::polylines(image, points, true, fillColor, 1);          // Purple for not visible triangles
            cv::fillConvexPoly(image, points, fillColor, cv::LINE_AA); // Fill not visible triangles
        }

        // Draw the triangles for visible vertices
        for (const auto &cara : caras)
        {
            std::vector<cv::Point> points;
            for (const auto &index : cara.indices)
            {
                points.push_back(imgpts[index]);
            }
            double intensity = calculateIllumination(vertices[cara.indices[0]], vertices[cara.indices[1]], vertices[cara.indices[2]], lightPos);
            cv::Scalar fillColor = cv::Scalar(0, 255, 0) * intensity;
            cv::polylines(image, points, true, fillColor, 1);          // Green for visible triangles
            cv::fillConvexPoly(image, points, fillColor, cv::LINE_AA); // Fill visible triangles
        }

        /*
        std::vector<cv::Point3f> closeToMidpoint;
        double threshold = 0.025; // Adjust this threshold as needed
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            const auto &vertex = vertices[i];
            const auto &pt = imgpts[i];
            if (std::fabs(vertex.z - midpointZ) < threshold)
            {
                closeToMidpoint.push_back(vertex);
                // Check if the point is in visibleVertices or notVisibleVertices
                if (std::find(visibleVertices.begin(), visibleVertices.end(), vertex) != visibleVertices.end())
                {
                    // cv::circle(image, pt, 3, cv::Scalar(255, 255, 255), -1); // White for points close to the midpoint in visibleVertices
                }
                else if (std::find(notVisibleVertices.begin(), notVisibleVertices.end(), vertex) != notVisibleVertices.end())
                {
                    // cv::circle(image, pt, 3, cv::Scalar(0, 165, 255), -1); // Orange for points close to the midpoint in notVisibleVertices
                }
            }
        }

        // Calculate the midpoint of Y for points close to Z midpoint
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        for (const auto &vertex : closeToMidpoint)
        {
            if (vertex.x < minX)
                minX = vertex.x;
            if (vertex.x > maxX)
                maxX = vertex.x;
        }
        // Calculate the midpoint of Y for points close to Z midpoint
        double midpointX = (minX + maxX) / 2.0;

        std::vector<cv::Point3f> notVisibleExtremes;
        std::vector<cv::Point3f> VisibleExtremes;
        for (size_t i = 0; i < closeToMidpoint.size(); ++i)
        {
            const auto &vertex = closeToMidpoint[i];
            if (vertex.x >= midpointX)
            {
                VisibleExtremes.push_back(vertex);
            }
            else
            {
                notVisibleExtremes.push_back(vertex);
            }
        }
        */
    /*
    std::vector<delaunay3D::Point<float>> verticesNewVisibleExtremes;
    for (const auto &pt : VisibleExtremes)
        verticesNewVisibleExtremes.emplace_back(delaunay3D::Point<float>(pt.x, pt.y, pt.z));
    std::vector<delaunay3D::Point<float>> verticesNewNotVisibleExtremes;
    for (const auto &pt : notVisibleExtremes)
        verticesNewNotVisibleExtremes.emplace_back(delaunay3D::Point<float>(pt.x, pt.y, pt.z));
    */
    // auto delaunayVisibleExtremes = triangulatesXZ(verticesNewVisibleExtremes);
    // auto delaunayNotVisibleExtremes = triangulatesXZ(verticesNewNotVisibleExtremes);
    // createCaras(verticesNew, delaunayVisibleExtremes, carasVisibleExtremes);
    // createCaras(verticesNew, delaunayNotVisibleExtremes, carasNotVisibleExtremes);

    // Draw the triangles for not visible vertices

    /*
    for (const auto &cara : carasVisibleExtremes)
    {
        std::vector<cv::Point> points;
        for (const auto &index : cara.indices)
        {
            points.push_back(imgpts[index]);
        }
        cv::polylines(image, points, true, cv::Scalar(0, 255, 255), 1); // Cyan for visible extremes
                                                                        // cv::fillConvexPoly(image, points, cv::Scalar(255, 0, 255), cv::LINE_AA); // Fill visible extremes
    }

    for (const auto &cara : carasNotVisibleExtremes)
    {
        std::vector<cv::Point> points;
        for (const auto &index : cara.indices)
        {
            points.push_back(imgpts[index]);
        }
        cv::polylines(image, points, true, cv::Scalar(255, 255, 0), 1); // Yellow for not visible extremes
        // cv::fillConvexPoly(image, points, cv::Scalar(0, 255, 0), cv::LINE_AA); // Fill not visible extremes
    }*/

protected:
    void printPoints2f(const std::vector<cv::Point2f> &points)
    {
        int contador = 0;
        for (const auto &point : points)
        {
            std::cout << ++contador << point << std::endl;
        }
    }
};