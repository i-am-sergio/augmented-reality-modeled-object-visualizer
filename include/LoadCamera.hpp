#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

using namespace cv;
using namespace std;
float markerLength = 0.1f;
float dx = -0.05f, dy = -0.05f, dz = 0.15f;

std::vector<cv::Point3f> getCubeVertices() {
    return {
        {dx, dy, dz}, {dx, dy + markerLength, dz}, {dx + markerLength, dy + markerLength, dz}, {dx + markerLength, dy, dz},
        {dx, dy, dz - markerLength}, {dx, dy + markerLength, dz - markerLength}, {dx + markerLength, dy + markerLength, dz - markerLength}, {dx + markerLength, dy, dz - markerLength}
    };
}

std::vector<cv::Point3f> getPrismVertices() {
    return {
        {dx, dy, dz}, {dx, dy + markerLength, dz}, {dx + markerLength, dy + markerLength, dz}, {dx + markerLength, dy, dz},
        {dx + 0.5f * markerLength, dy + 0.5f * markerLength, dz - markerLength}, {dx + 0.5f * markerLength, dy + 0.5f * markerLength, dz - markerLength}, {dx + 1.5f * markerLength, dy + 1.5f * markerLength, dz - markerLength}, {dx + 1.5f * markerLength, dy + 1.5f * markerLength, dz - markerLength}
    };
}

void drawCube(cv::Mat& frame, const std::vector<cv::Point2f>& imgpts) {
    // Ensure imgpts size is correct
    if (imgpts.size() != 8) return;

    // Define contours for the cube
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(std::vector<cv::Point>(imgpts.begin(), imgpts.begin() + 4));
    contours.push_back(std::vector<cv::Point>(imgpts.begin() + 4, imgpts.end()));

    // Draw ground floor in green
    cv::drawContours(frame, contours, 0, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

    // Draw pillars in blue
    for (int i = 0; i < 4; ++i) {
        cv::line(frame, imgpts[i], imgpts[i + 4], cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    }

    // Draw top layer in red
    cv::drawContours(frame, contours, 1, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
}

void drawPrism(cv::Mat& frame, const std::vector<cv::Point2f>& imgpts) {
    // Ensure imgpts size is correct
    if (imgpts.size() != 8) return;

    // Define contours for the prism
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(std::vector<cv::Point>(imgpts.begin(), imgpts.begin() + 4));
    contours.push_back(std::vector<cv::Point>(imgpts.begin() + 4, imgpts.end()));

    // Draw ground floor in yellow
    cv::drawContours(frame, contours, 0, cv::Scalar(0, 255, 255), 3, cv::LINE_AA);

    // Draw pillars in cyan
    for (int i = 0; i < 4; ++i) {
        cv::line(frame, imgpts[i], imgpts[i + 4], cv::Scalar(255, 255, 0), 3, cv::LINE_AA);
    }

    // Draw top layer in magenta
    cv::drawContours(frame, contours, 1, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
}


class LoadCamera {
    private: 
        cv::VideoCapture cap;
        cv::Mat frame;
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        float markerLength; // Marker length in meters
        cv::Ptr<cv::aruco::Dictionary> arucoDict;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

    public:
        LoadCamera(){
            // Default camera calibration parameters (needs to be calibrated for actual use)
            cameraMatrix = (cv::Mat_<float>(3, 3) << 800, 0, 320, 0, 800, 240, 0, 0, 1);
            distCoeffs = cv::Mat::zeros(5, 1, CV_32F);
            arucoDict = new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
            detectorParams = new cv::aruco::DetectorParameters();
            markerLength = 0.1f;
        }

        ~LoadCamera(){
            if (cap.isOpened()){
                cap.release();
            }
        }
        
        bool openCamera(int cameraIndex = 0){
            cap.open(cameraIndex);
            return cap.isOpened();
        }

        void showCamera(){

            while (true){
                this->cap >> frame;
                if (frame.empty()) break;

                // Convert frame to grayscale
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                // Detect markers
                std::vector<std::vector<cv::Point2f>> markerCorners;
                std::vector<int> markerIDs;
                std::vector<std::vector<cv::Point2f>> reject;

                cv::aruco::detectMarkers(gray, arucoDict, markerCorners, markerIDs, detectorParams, reject);

                if(!markerIDs.empty())
                {
                    // Draw detected markers and display their IDs
                    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIDs);
                    
                    // Estimate pose of each marker
                    std::vector<cv::Vec3d> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

                    for (size_t i = 0; i < markerIDs.size(); i++){
                        // Draw axis for each marker
                        cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength);

                        std::cout << "len marker ids: " << markerIDs.size() << std::endl;
                        std::cout << "Rotation vector: " << rvecs[i] << std::endl;
                        std::cout << "Translation vector: " << tvecs[i] << std::endl;
                        std::cout << "ids: " << markerIDs[0] << std::endl;

                        // Select the vertices based on the marker ID
                        if (markerIDs[i] == 25) {
                            auto vertices = getCubeVertices();
                            std::vector<cv::Point2f> imgpts;
                            cv::projectPoints(vertices, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imgpts);
                            drawCube(frame, imgpts);
                        } else if (markerIDs[i] == 30) {
                            auto vertices = getPrismVertices();
                            std::vector<cv::Point2f> imgpts;
                            cv::projectPoints(vertices, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imgpts);
                            drawPrism(frame, imgpts);
                        }
                    }
                }

                cv::imshow("Camera", frame);

                // stop to press 'q' or 'esc'
                if (cv::waitKey(10) == 27 || cv::waitKey(10) == 'q'){
                    break;
                }
                // stop to clicked x button
                if (cv::getWindowProperty("Camera", cv::WND_PROP_AUTOSIZE) == -1){
                    break;
                }
            }

        }
};