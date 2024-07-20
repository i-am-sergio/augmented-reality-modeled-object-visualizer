#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "Load3DModel.hpp"
#include "Delaunay.hpp"
#include <vector>
#include <cmath>
using namespace cv;
using namespace std;
using namespace delaunay;

double calculateDistance(const cv::Point &p1, const cv::Point &p2)
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
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

        std::vector<delaunay::Point<float>> delaunayPoints;
        for (const auto &pt : imgpts)
        {
            delaunayPoints.emplace_back(pt.x, pt.y);
        }

        auto triangulation = triangulate(delaunayPoints);

        for (const auto &pt : imgpts)
        {
            cv::circle(image, pt, 2, cv::Scalar(22, 21, 250), -1);
        }

        double maxDistance = 0.0;

        for (const auto &e : triangulation.edges)
        {
            double distance = calculateDistance(cv::Point(e.p0.x, e.p0.y), cv::Point(e.p1.x, e.p1.y));
            if (distance > maxDistance)
            {
                maxDistance = distance;
            }
        }

        double maxDistanceThreshold = maxDistance * maxDistancePercentage;

        for (const auto &e : triangulation.edges)
        {
            double distance = calculateDistance(cv::Point(e.p0.x, e.p0.y), cv::Point(e.p1.x, e.p1.y));
            if (distance <= maxDistanceThreshold)
            {
                cv::line(image, cv::Point(e.p0.x, e.p0.y), cv::Point(e.p1.x, e.p1.y), cv::Scalar(193, 107, 255), 1);
            }
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