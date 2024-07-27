
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

public:
    ObjectProjection(const std::vector<cv::Point3f> &vertices, const std::vector<cv::Point3f> &normals, const std::vector<cv::Point2f> &texCoords, const std::vector<Face> &faces, float maxDistancePercentage)
        : vertices(vertices), normals(normals), texCoords(texCoords), faces(faces), maxDistancePercentage(maxDistancePercentage) {}
    ObjectProjection(const std::vector<cv::Point3f> &vertices, const std::vector<cv::Point3f> &normals, const std::vector<cv::Point2f> &texCoords, const std::vector<CaraNew> &carass, float maxDistancePercentage)
        : vertices(vertices), normals(normals), texCoords(texCoords), caras(carass), maxDistancePercentage(maxDistancePercentage) {}

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
    void drawObject(cv::Mat &image, cv::Vec3d rvec, cv::Vec3d tvec, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                cv::Vec3d additionalRotation = cv::Vec3d(0, 0, 0), const AnimationConfig &animationConfig = AnimationConfig())
{
    // Convert rotation vector to rotation matrix
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    cv::Mat addRotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(additionalRotation[0]), -sin(additionalRotation[0]), 0, sin(additionalRotation[0]), cos(additionalRotation[0]));
    cv::Mat addRotY = (cv::Mat_<double>(3, 3) << cos(additionalRotation[1]), 0, sin(additionalRotation[1]), 0, 1, 0, -sin(additionalRotation[1]), 0, cos(additionalRotation[1]));
    cv::Mat addRotZ = (cv::Mat_<double>(3, 3) << cos(additionalRotation[2]), -sin(additionalRotation[2]), 0, sin(additionalRotation[2]), cos(additionalRotation[2]), 0, 0, 0, 1);

    if (animationConfig.rotate)
    {
        rmat = addRotX * addRotY * rmat;
    }
    if (animationConfig.spin)
    {
        rmat = addRotZ * rmat;
    }
    if (animationConfig.iluminate)
    {
        rmat = rmat * addRotX * addRotY;
    }
    else
    {
        cv::Rodrigues(rmat, rvec);
    }

    // Update the translation vector with the xTranslation
    tvec[0] += animationConfig.xTranslation;
    tvec[1] += animationConfig.yTranslation;
    tvec[2] += animationConfig.scaleObject;
    cv::Mat lightMat = rmat.inv();              // Get the inverse of the rotation matrix
    cv::Point3f lightSourcePosition(0, 0, 0.4); // Light source position in object space
    cv::Point3f lightPos = cv::Point3f(lightMat.at<double>(0, 0) * lightSourcePosition.x + lightMat.at<double>(0, 1) * lightSourcePosition.y + lightMat.at<double>(0, 2) * lightSourcePosition.z,
                                       lightMat.at<double>(1, 0) * lightSourcePosition.x + lightMat.at<double>(1, 1) * lightSourcePosition.y + lightMat.at<double>(1, 2) * lightSourcePosition.z,
                                       lightMat.at<double>(2, 0) * lightSourcePosition.x + lightMat.at<double>(2, 1) * lightSourcePosition.y + lightMat.at<double>(2, 2) * lightSourcePosition.z);

    // Project light position onto the image plane
    std::vector<cv::Point2f> lightSourceImgPts;
    cv::projectPoints(std::vector<cv::Point3f>{lightPos}, rvec, tvec, cameraMatrix, distCoeffs, lightSourceImgPts);

    // Project points
    std::vector<cv::Point2f> imgpts(vertices.size());
    cv::projectPoints(vertices, rvec, tvec, cameraMatrix, distCoeffs, imgpts);

    // Draw the light source on the image
    for (const auto &pt : lightSourceImgPts)
    {
        cv::circle(image, pt, 10, cv::Scalar(255, 255, 255), -1);
    }

    auto calculateIllumination = [&](const cv::Point3f &p0, const cv::Point3f &p1, const cv::Point3f &p2, const cv::Point3f &lightPos)
    {
        // Calcular la normal del triángulo
        cv::Point3f edge1 = p1 - p0;
        cv::Point3f edge2 = p2 - p0;
        cv::Point3f normal = edge1.cross(edge2);
        double normLength = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        normal /= normLength; // Normalizar la normal

        // Calcular el vector de dirección de la luz desde la posición de la luz
        cv::Point3f centroid = (p0 + p1 + p2) / 3.0;
        cv::Point3f lightDir = lightPos - centroid;
        double lightDirLength = std::sqrt(lightDir.x * lightDir.x + lightDir.y * lightDir.y + lightDir.z * lightDir.z);
        lightDir /= lightDirLength; // Normalizar la dirección de la luz

        // Calcular el producto punto con la dirección de la luz
        double dotProduct = normal.dot(lightDir);

        // Limitar el producto punto para que esté entre 0 y 1
        double intensity = std::max(0.0, std::min(1.0, dotProduct));

        return intensity;
    };

    for (const auto &face : faces)
    {
        std::vector<cv::Point> points;
        for (const auto &vertex : face.vertices)
        {
            points.push_back(imgpts[vertex - 1]);
        }
        double intensity = calculateIllumination(vertices[face.vertices[0] - 1], vertices[face.vertices[1] - 1], vertices[face.vertices[2] - 1], lightPos);

        // Adjust face color based on intensity
        cv::Scalar fillColor = cv::Scalar(255, 179, 153) * intensity;
        cv::polylines(image, points, true, fillColor, 1);
        cv::fillConvexPoly(image, points, fillColor, cv::LINE_AA);
    }
}


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