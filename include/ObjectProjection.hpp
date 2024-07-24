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

double calculateDistance(const cv::Point &p1, const cv::Point &p2)
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}
struct Cara
{
    std::vector<int> indices;
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
    float maxDistancePercentage;

public:
    ObjectProjection(const std::vector<cv::Point3f> &vertices, const std::vector<cv::Point3f> &normals, const std::vector<cv::Point2f> &texCoords, const std::vector<Face> &faces, float maxDistancePercentage)
        : vertices(vertices), normals(normals), texCoords(texCoords), faces(faces), maxDistancePercentage(maxDistancePercentage) {}

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

    void drawObject(cv::Mat &image, cv::Vec3d rvec, cv::Vec3d tvec, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, cv::Vec3d additionalRotation = cv::Vec3d(0, 0, 0))
    {
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);

        cv::Mat addRotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(additionalRotation[0]), -sin(additionalRotation[0]), 0, sin(additionalRotation[0]), cos(additionalRotation[0]));
        cv::Mat addRotY = (cv::Mat_<double>(3, 3) << cos(additionalRotation[1]), 0, sin(additionalRotation[1]), 0, 1, 0, -sin(additionalRotation[1]), 0, cos(additionalRotation[1]));

        rmat = rmat * addRotX * addRotY;

        cv::Rodrigues(rmat, rvec);

        std::vector<cv::Point2f> imgpts;
        cv::projectPoints(vertices, rvec, tvec, cameraMatrix, distCoeffs, imgpts);

        std::vector<delaunay3D::Point<float>> verticesNew;
        for (const auto &pt : vertices)
        {
            verticesNew.emplace_back(delaunay3D::Point<float>(pt.x, pt.y, pt.z));
        }
        auto delaunay = triangulates(verticesNew);

        std::vector<Cara> caras;
        createCaras(verticesNew, delaunay, caras);
        for (const auto &cara : caras)
        {
            std::vector<cv::Point> points;
            for (const auto &index : cara.indices)
            {
                points.push_back(imgpts[index]);
            }
            cv::fillConvexPoly(image, points, cv::Scalar(255, 179, 153));
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