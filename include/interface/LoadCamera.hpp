#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include "ObjectProjection.hpp"

using namespace cv;
using namespace std;

class LoadCamera
{
private:
    cv::VideoCapture cap;
    cv::Mat frame;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    float markerLength; // Marker length in meters
    cv::Ptr<cv::aruco::Dictionary> arucoDict;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    vector<ObjectProjection> objectsProjections;

    // Mouse state
    bool isDragging = false;
    cv::Point lastMousePos;
    cv::Vec3d rotation;

    AnimationConfig animationConfig;

    vector<cv::Scalar> baseColors = {
        cv::Scalar(255, 0, 0),     // Blue
        cv::Scalar(0, 255, 0),     // Green
        cv::Scalar(0, 0, 255),     // Red
        cv::Scalar(176, 109, 242), // Purple
        cv::Scalar(154, 250, 65),  // Aqua
        cv::Scalar(255, 229, 84),  // Sky Blue
        cv::Scalar(0, 255, 255),   // Yellow
        cv::Scalar(255, 0, 255),   // Orange
    };
    int currentColorIndex = 0;

public:
    LoadCamera(vector<ObjectProjection> &objectsProjections);
    ~LoadCamera();
    bool openCamera(int cameraIndex = 0);
    static void onMouse(int event, int x, int y, int, void *userdata);
    void showCamera();

private:
    std::vector<cv::Point3f> getCubeVertices();
    std::vector<cv::Point3f> getPrismVertices();
    void drawShape(cv::Mat &frame, const std::vector<cv::Point2f> &imgpts);
};
