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

    void drawObject(cv::Mat& frame, const cv::Vec3d& rvec, const cv::Vec3d& tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,double movement = 0.0) {
        // Ajustar la posición en el eje Z
        cv::Vec3d adjusted_tvec = tvec + cv::Vec3d(0, 0, movement);
        
        // Proyectar los vértices del objeto
        std::vector<cv::Point2f> imgpts;
        cv::projectPoints(vertices, rvec, adjusted_tvec, cameraMatrix, distCoeffs, imgpts);
        
        // Dibujar los vértices como puntos
        for (const auto& pt : imgpts) {
            cv::circle(frame, pt, 3, cv::Scalar(22, 21, 250), -1);
        }

        // Dibujar las caras
        for (const auto& face : faces) {
            std::vector<cv::Point> points;
            for (const auto& vertex : face.vertices) {
                points.push_back(imgpts[vertex - 1]);
            }
            // cv::polylines(frame, points, true, cv::Scalar(255, 255, 0), 2);
            // Dibujar el polígono sólido
            // cv::fillPoly(frame, std::vector<std::vector<cv::Point>>{points}, cv::Scalar(255, 179, 153)); // Color en formato BGR
            // Use fillConvexPoly to fill the polygon
            cv::fillConvexPoly(frame, points, cv::Scalar(255, 179, 153)); // Color en formato BGR
        }
    }

    void drawObject(cv::Mat& image, cv::Vec3d rvec, cv::Vec3d tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Vec3d additionalRotation = cv::Vec3d(0, 0, 0)) {
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);

        // Apply additional rotation
        cv::Mat addRotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(additionalRotation[0]), -sin(additionalRotation[0]), 0, sin(additionalRotation[0]), cos(additionalRotation[0]));
        cv::Mat addRotY = (cv::Mat_<double>(3, 3) << cos(additionalRotation[1]), 0, sin(additionalRotation[1]), 0, 1, 0, -sin(additionalRotation[1]), 0, cos(additionalRotation[1]));

        rmat = rmat * addRotX * addRotY;

        cv::Rodrigues(rmat, rvec);

        // Draw points
        std::vector<cv::Point2f> imgpts;
        cv::projectPoints(vertices, rvec, tvec, cameraMatrix, distCoeffs, imgpts);
        for (const auto& pt : imgpts) {
            cv::circle(image, pt, 3, cv::Scalar(22, 21, 250), -1);
        }
    }

};
