#include "KMeansReduction.hpp"
#include <algorithm>
#include <numeric>

KMeansReduction::KMeansReduction(const std::vector<cv::Point3f> &vertices,
                                 const std::vector<cv::Point3f> &normals,
                                 const std::vector<cv::Point2f> &texCoords,
                                 const std::vector<Face> &faces)
    : vertices(vertices), normals(normals), texCoords(texCoords), faces(faces) {}

void KMeansReduction::applyReduction(float reductionFactor)
{
    if (vertices.empty())
        return;

    int numClusters = static_cast<int>(vertices.size() * reductionFactor);

    // K-means clustering algorithm
    cv::Mat points(vertices.size(), 3, CV_32F);
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        points.at<float>(i, 0) = vertices[i].x;
        points.at<float>(i, 1) = vertices[i].y;
        points.at<float>(i, 2) = vertices[i].z;
    }

    cv::Mat labels, centers;
    cv::kmeans(points, numClusters, labels,
               cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
               3, cv::KMEANS_PP_CENTERS, centers);

    std::vector<cv::Point3f> reducedVertices(numClusters);
    std::vector<cv::Point3f> reducedNormals(normals.size() ? numClusters : 0);
    std::vector<cv::Point2f> reducedTexCoords(texCoords.size() ? numClusters : 0);
    std::vector<int> clusterCounts(numClusters, 0);

    for (size_t i = 0; i < vertices.size(); ++i)
    {
        int clusterIdx = labels.at<int>(i);
        reducedVertices[clusterIdx] += vertices[i];
        if (!normals.empty())
            reducedNormals[clusterIdx] += normals[i];
        if (!texCoords.empty())
            reducedTexCoords[clusterIdx] += texCoords[i];
        clusterCounts[clusterIdx]++;
    }

    for (int i = 0; i < numClusters; ++i)
    {
        reducedVertices[i] /= static_cast<float>(clusterCounts[i]);
        if (!normals.empty())
            reducedNormals[i] /= static_cast<float>(clusterCounts[i]);
        if (!texCoords.empty())
            reducedTexCoords[i] /= static_cast<float>(clusterCounts[i]);
    }

    vertices = std::move(reducedVertices);
    if (!normals.empty())
        normals = std::move(reducedNormals);
    if (!texCoords.empty())
        texCoords = std::move(reducedTexCoords);

    // Updating faces to reference reduced vertices
    for (auto &face : faces)
        for (auto &vertexIndex : face.vertices)
            vertexIndex = labels.at<int>(vertexIndex - 1) + 1;

    // Removing duplicate faces
    std::sort(faces.begin(), faces.end(), [](const Face &a, const Face &b)
              { return a.vertices < b.vertices; });

    faces.erase(std::unique(faces.begin(), faces.end(), [](const Face &a, const Face &b)
                            { return a.vertices == b.vertices; }),
                faces.end());
}

std::vector<cv::Point3f> KMeansReduction::getVertices() const
{
    return vertices;
}

std::vector<cv::Point3f> KMeansReduction::getNormals() const
{
    return normals;
}

std::vector<cv::Point2f> KMeansReduction::getTexCoords() const
{
    return texCoords;
}

std::vector<Face> KMeansReduction::getFaces() const
{
    return faces;
}