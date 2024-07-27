#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "Load3DModel.hpp"
using namespace std;

// Function to perform Delaunay triangulation
void performDelaunayTriangulation(const std::vector<cv::Point2f>& points, std::vector<cv::Vec6f>& triangleList) {
    // Define the rectangle that bounds the points
    cv::Rect rect(0, 0, 500, 500);

    // Create an instance of Subdiv2D
    cv::Subdiv2D subdiv(rect);

    // Insert points into subdiv
    for (const auto& point : points) {
        subdiv.insert(point);
    }

    // Get the list of triangles
    subdiv.getTriangleList(triangleList);
}

int main() {
    // Load 3D model
    Load3DModel model("models/rat.obj", true, 0, 0, 0.15f, 0.05, 0.004f);
    // vector<cv::Point3f> vertices = model.getVertices();

    // Example points
    std::vector<cv::Point3f> vertices = { {100, 100, 0}, {200, 150, 0}, {250, 250, 0}, {300, 100, 0}, {350, 300, 0} };
    
    // Project 3D points to 2D (ignoring z)
    std::vector<cv::Point2f> points2D;
    for (const auto& vertex : vertices) {
        points2D.emplace_back(vertex.x, vertex.y);
    }

    // Perform Delaunay triangulation
    std::vector<cv::Vec6f> triangleList;
    performDelaunayTriangulation(points2D, triangleList);

    // Create an image to draw the results
    cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);

    // Draw the triangles
    for (size_t i = 0; i < triangleList.size(); i++) {
        cv::Vec6f t = triangleList[i];
        cv::Point pt1(cvRound(t[0]), cvRound(t[1]));
        cv::Point pt2(cvRound(t[2]), cvRound(t[3]));
        cv::Point pt3(cvRound(t[4]), cvRound(t[5]));

        cv::line(img, pt1, pt2, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 0);
        cv::line(img, pt2, pt3, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 0);
        cv::line(img, pt3, pt1, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 0);
    }

    // Show the image
    cv::imshow("Delaunay Triangulation", img);
    cv::waitKey(0);

    return 0;
}
