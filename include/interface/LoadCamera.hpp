#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <random> // Include this header for default_random_engine
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
    int numPartitions = 8;     // Number of circle partitions
    float radiusCircle = 0.4f; // Scale factor for the circle
    bool spin = false;         // Flag to indicate if the bottle should spin
    float bottleAngle = 0;     // Current angle of the bottle
    float spinSpeed = 0;       // Spin speed of the bottle
    // vector<string> sectionTexts = {"1", "2", "3", "4", "5", "6", "7", "8"}; // Texts for each section

    struct RGBColor
    {
        int r, g, b;
    };

    vector<cv::Scalar> sectionColors;
    bool colorsInitialized = false;

    // Random color generator
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution;

    // Text for sections
    vector<string> sectionTexts;

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
    void initializeSectionColors(int numSections);
    void drawCircleOnMarker(cv::Mat &frame, const cv::Vec3d &rvec, const cv::Vec3d &tvec, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, float radius, int numSections);
    void startSpin();
    void spinBottle();
};
