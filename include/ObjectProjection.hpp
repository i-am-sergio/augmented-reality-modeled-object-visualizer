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
        // Convert rotation vector to rotation matrix
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        cv::Mat addRotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(additionalRotation[0]), -sin(additionalRotation[0]), 0, sin(additionalRotation[0]), cos(additionalRotation[0]));
        cv::Mat addRotY = (cv::Mat_<double>(3, 3) << cos(additionalRotation[1]), 0, sin(additionalRotation[1]), 0, 1, 0, -sin(additionalRotation[1]), 0, cos(additionalRotation[1]));

        rmat = rmat * addRotX * addRotY;
        cv::Rodrigues(rmat, rvec);

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

        auto delaunay = triangulates(verticesNewVisible);
        auto delaunayNotVisible = triangulates(verticesNewNotVisible);

        std::vector<Cara> caras, carasNotVisible;
        createCaras(verticesNew, delaunay, caras);
        createCaras(verticesNew, delaunayNotVisible, carasNotVisible);

        // Draw the triangles for visible vertices
        for (const auto &cara : caras)
        {
            std::vector<cv::Point> points;
            for (const auto &index : cara.indices)
            {
                points.push_back(imgpts[index]);
            }
            cv::polylines(image, points, true, cv::Scalar(0, 255, 0), 1); // Green for visible triangles
            // cv::fillConvexPoly(image, points, cv::Scalar(0, 255, 0), cv::LINE_AA); // Fill visible triangles
        }

        // Draw the triangles for not visible vertices
        for (const auto &cara : carasNotVisible)
        {
            std::vector<cv::Point> points;
            for (const auto &index : cara.indices)
            {
                points.push_back(imgpts[index]);
            }
            cv::polylines(image, points, true, cv::Scalar(255, 0, 255), 1); // Purple for not visible triangles
            // cv::fillConvexPoly(image, points, cv::Scalar(255, 0, 255), cv::LINE_AA); // Fill not visible triangles
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