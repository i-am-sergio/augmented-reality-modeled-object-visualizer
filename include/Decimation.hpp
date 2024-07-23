#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <random>
#include "Load3DModel.hpp"

class Decimation {
private:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texCoords;
    std::vector<Face> faces;

public:
    Decimation(const std::vector<cv::Point3f>& vertices,
               const std::vector<cv::Point3f>& normals,
               const std::vector<cv::Point2f>& texCoords,
               const std::vector<Face>& faces)
        : vertices(vertices), normals(normals), texCoords(texCoords), faces(faces) {}

    void applyDecimation(float reductionFactor) {
        int targetSize = static_cast<int>(vertices.size() * reductionFactor);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, vertices.size() - 1);

        std::vector<int> indices(vertices.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), gen);

        indices.resize(targetSize);

        std::sort(indices.begin(), indices.end());

        std::vector<cv::Point3f> reducedVertices(targetSize);
        std::vector<cv::Point3f> reducedNormals(normals.size() ? targetSize : 0);
        std::vector<cv::Point2f> reducedTexCoords(texCoords.size() ? targetSize : 0);

        for (int i = 0; i < targetSize; ++i) {
            reducedVertices[i] = vertices[indices[i]];
            if (!normals.empty()) {
                reducedNormals[i] = normals[indices[i]];
            }
            if (!texCoords.empty()) {
                reducedTexCoords[i] = texCoords[indices[i]];
            }
        }

        vertices = std::move(reducedVertices);
        if (!normals.empty()) {
            normals = std::move(reducedNormals);
        }
        if (!texCoords.empty()) {
            texCoords = std::move(reducedTexCoords);
        }

        // Updating faces to reference reduced vertices
        for (auto& face : faces) {
            for (auto& vertexIndex : face.vertices) {
                auto it = std::find(indices.begin(), indices.end(), vertexIndex - 1);
                if (it != indices.end()) {
                    vertexIndex = std::distance(indices.begin(), it) + 1;
                } else {
                    vertexIndex = 0; // Mark invalid vertex
                }
            }

            face.vertices.erase(std::remove(face.vertices.begin(), face.vertices.end(), 0), face.vertices.end());
        }

        faces.erase(std::remove_if(faces.begin(), faces.end(), [](const Face& face) {
            return face.vertices.empty();
        }), faces.end());
    }

    std::vector<cv::Point3f> getVertices() const { return vertices; }
    std::vector<cv::Point3f> getNormals() const { return normals; }
    std::vector<cv::Point2f> getTexCoords() const { return texCoords; }
    std::vector<Face> getFaces() const { return faces; }
};
