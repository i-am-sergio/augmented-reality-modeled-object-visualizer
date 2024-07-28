#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "Load3DModel.hpp"
#include <vector>
#include <cmath>
#include <unordered_map>
#include <functional>

using namespace cv;
using namespace std;

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
    cv::Scalar baseColor;
};

class ObjectProjection
{
private:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texCoords;
    std::vector<Face> faces;
    float maxDistancePercentage;
    double lightSpeed;  // Speed of light movement in radians per second
    double lightRadius; // Radius of light orbit
    double startTime;   // Time variable to keep track of the animation

public:
    ObjectProjection(const std::vector<cv::Point3f> &vertices,
                     const std::vector<cv::Point3f> &normals,
                     const std::vector<cv::Point2f> &texCoords,
                     const std::vector<Face> &faces,
                     float maxDistancePercentage, double lightSpeed = 1.0,
                     double lightRadius = 0.4);
    void drawObject(cv::Mat &image, cv::Vec3d rvec,
                    cv::Vec3d tvec, const cv::Mat &cameraMatrix,
                    const cv::Mat &distCoeffs,
                    cv::Vec3d additionalRotation = cv::Vec3d(0, 0, 0),
                    const AnimationConfig &animationConfig = AnimationConfig());

protected:
    void printPoints2f(const std::vector<cv::Point2f> &points);
};