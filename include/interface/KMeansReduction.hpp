#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "Load3DModel.hpp"

class KMeansReduction
{
private:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texCoords;
    std::vector<Face> faces;

public:
    KMeansReduction(const std::vector<cv::Point3f> &vertices,
                    const std::vector<cv::Point3f> &normals,
                    const std::vector<cv::Point2f> &texCoords,
                    const std::vector<Face> &faces);

    void applyReduction(float reductionFactor);

    std::vector<cv::Point3f> getVertices() const;
    std::vector<cv::Point3f> getNormals() const;
    std::vector<cv::Point2f> getTexCoords() const;
    std::vector<Face> getFaces() const;
};
