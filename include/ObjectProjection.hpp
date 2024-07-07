#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "Load3DModel.hpp"
#include <vector>

using namespace cv;
using namespace std;

class ObjectProjection {
private:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texCoords;
    std::vector<Face> faces;

public:
    ObjectProjection(const std::vector<cv::Point3f>& vertices, const std::vector<cv::Point3f>& normals, const std::vector<cv::Point2f>& texCoords, const std::vector<Face>& faces) {
        this->vertices = vertices;
        this->normals = normals;
        this->texCoords = texCoords;
        this->faces = faces;
    }

    void drawObject(cv::Mat& frame, const std::vector<cv::Point2f>& imgpts) {
        // Draw the vertices as points
        for (const auto& pt : imgpts) {
            cv::circle(frame, pt, 3, cv::Scalar(0, 0, 255), -1);
        }

        // Draw the faces
        for (const auto& face : faces) {
            std::vector<cv::Point> points;
            for (const auto& vertex : face.vertices) {
                points.push_back(imgpts[vertex - 1]);
            }
            cv::polylines(frame, points, true, cv::Scalar(255, 255, 0), 2);
        }
    }

    void processFrame(cv::Mat& frame, const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, float markerLength) {
        if (ids.empty()) return;

        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); ++i) {
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength);

            if (ids[i] == 35) {
                std::vector<cv::Point2f> imgpts;
                cv::projectPoints(vertices, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imgpts);
                drawObject(frame, imgpts);
            }
        }
    }
};
