#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <filesystem>
#include <string>

template <int dictionaryId>
class PatternsGenerator
{
private:
    int imgSize;
    int markerSize;
    cv::Ptr<cv::aruco::Dictionary> arucoDict;
    std::vector<int> markerIds;

    cv::Mat centerMarkerOnWhiteImage(const cv::Mat &marker)
    {
        cv::Mat white_image = cv::Mat::ones(imgSize, imgSize, CV_8UC3) * 255;
        cv::Mat markerColor;
        cv::cvtColor(marker, markerColor, cv::COLOR_GRAY2BGR);
        int top_left_x = (imgSize - markerSize) / 2;
        int top_left_y = (imgSize - markerSize) / 2;
        markerColor.copyTo(white_image(cv::Rect(top_left_x, top_left_y, markerSize, markerSize)));
        return white_image;
    }

    cv::Mat centerMarkerOnPastelBackground(const cv::Mat &marker, cv::RNG &rng)
    {
        cv::Mat pastel_background = cv::Mat::zeros(imgSize, imgSize, CV_8UC3);
        cv::Scalar background_color = cv::Scalar(rng.uniform(150, 256), rng.uniform(150, 256), rng.uniform(150, 256));
        pastel_background.setTo(background_color);
        cv::Mat marker_bw;
        cv::cvtColor(marker, marker_bw, cv::COLOR_GRAY2BGR);
        int top_left_x = (imgSize - markerSize) / 2;
        int top_left_y = (imgSize - markerSize) / 2;
        marker_bw.copyTo(pastel_background(cv::Rect(top_left_x, top_left_y, markerSize, markerSize)));
        return pastel_background;
    }

public:
    PatternsGenerator(int imgSize, int markerSize, const std::vector<int> &ids)
    {
        this->imgSize = imgSize;
        this->markerSize = markerSize;
        this->markerIds = ids;
        this->arucoDict = new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(dictionaryId));
    }

    void generateMarkers()
    {
        cv::RNG rng(12345);
        for (int markerId : markerIds)
        {
            cv::Mat marker;
            cv::aruco::generateImageMarker(*arucoDict, markerId, markerSize, marker);
            cv::Mat pastel_marker = centerMarkerOnPastelBackground(marker, rng);
            if (!std::filesystem::exists("patterns"))
                std::filesystem::create_directory("patterns");
            std::string filename = "patterns/marker" + std::to_string(markerId) + ".png";
            cv::imwrite(filename, pastel_marker);
        }
    }

    void generateFourMarkersInOneImage()
    {
        cv::Mat combined_image = cv::Mat::ones(imgSize * 2, imgSize * 2, CV_8UC3) * 255;
        cv::RNG rng(12345);
        for (size_t i = 0; i < std::min<size_t>(4, markerIds.size()); ++i)
        {
            int markerId = markerIds[i];
            cv::Mat marker;
            cv::aruco::generateImageMarker(*arucoDict, markerId, markerSize, marker);
            cv::Mat pastel_marker = centerMarkerOnPastelBackground(marker, rng);
            int row = i / 2;
            int col = i % 2;
            cv::Mat roi = combined_image(cv::Rect(col * imgSize, row * imgSize, imgSize, imgSize));
            pastel_marker.copyTo(roi);
        }
        if (!std::filesystem::exists("patterns"))
            std::filesystem::create_directory("patterns");
        std::string filename = "patterns/four_markers.png";
        cv::imwrite(filename, combined_image);
    }

    void generateFourMarkersInRowRectangularImage()
    {
        cv::Mat combined_image = cv::Mat::ones(imgSize, imgSize * 4, CV_8UC3) * 255;
        cv::RNG rng(12345);
        for (size_t i = 0; i < std::min<size_t>(4, markerIds.size()); ++i)
        {
            int markerId = markerIds[i];
            cv::Mat marker;
            cv::aruco::generateImageMarker(*arucoDict, markerId, markerSize, marker);
            cv::Mat pastel_marker = centerMarkerOnPastelBackground(marker, rng);
            cv::Mat roi = combined_image(cv::Rect(i * imgSize, 0, imgSize, imgSize));
            pastel_marker.copyTo(roi);
        }
        if (!std::filesystem::exists("patterns"))
            std::filesystem::create_directory("patterns");
        std::string filename = "patterns/four_markers_row.png";
        cv::imwrite(filename, combined_image);
    }
};
