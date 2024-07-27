#include <iostream>
#include <opencv2/opencv.hpp>
#include "LoadCamera.hpp"
#include "Load3DModel.hpp"
#include "ObjectProjection.hpp"
#include "Decimation.hpp"
#include "KMeansReduction.hpp"
#include "Delaunay2D.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
using namespace std;
using namespace delaunay2d;

struct FlattenedPoint
{
    cv::Point3f originalPoint;
    cv::Point2f flattenedPoint;
    size_t index;
};
// Función para encontrar el punto medio más alto en z
cv::Point3f findMedianPointZ(const std::vector<cv::Point3f> &points)
{
    // Encontrar el punto con el valor z más alto
    cv::Point3f medianPoint = points[0];
    for (const auto &pt : points)
    {
        if (pt.z > medianPoint.z)
        {
            medianPoint = pt;
        }
    }
    return medianPoint;
}

cv::Point3f findMedianPointY(const std::vector<cv::Point3f> &points)
{
    // Encontrar el punto con el valor y más alto
    cv::Point3f medianPoint = points[0];
    for (const auto &pt : points)
    {
        if (pt.y > medianPoint.y)
        {
            medianPoint = pt;
        }
    }
    return medianPoint;
}

cv::Point3f findMedianPointX(const std::vector<cv::Point3f> &points)
{
    // Encontrar el punto con el valor y más alto
    cv::Point3f medianPoint = points[0];
    for (const auto &pt : points)
    {
        if (pt.x > medianPoint.x)
        {
            medianPoint = pt;
        }
    }
    return medianPoint;
}

// Función para aplanar los puntos 3D en un plano 2D con distribución circular
std::vector<FlattenedPoint> flattenPointsXY(const std::vector<cv::Point3f> &points)
{
    std::vector<FlattenedPoint> flattenedPoints;
    // Encontrar el punto medio más alto en z
    cv::Point3f medianPoint = findMedianPointZ(points);

    // Proyectar los puntos en el plano 2D (x, y) y ajustar según z
    for (const auto &pt : points)
    {
        FlattenedPoint flattenedPoint;
        flattenedPoint.originalPoint = pt;
        flattenedPoint.index = pt.x; // Assuming index corresponds to x here, but it can be updated as needed

        double scale;
        if (pt.z == medianPoint.z)
        {
            flattenedPoint.flattenedPoint = cv::Point2f(pt.x, pt.y); // Sin ajuste si z es igual
        }
        else
        {
            // Calculate circular position
            double zDiff = medianPoint.z - pt.z;
            double radius = zDiff * 0.1; // Adjust the scale factor (0.1) to change how the circle grows

            // Calculate angle in the circle based on the point's x and y
            double angle = std::atan2(pt.y - medianPoint.y, pt.x - medianPoint.x);
            double newX = medianPoint.x + radius * std::cos(angle);
            double newY = medianPoint.y + radius * std::sin(angle);
            flattenedPoint.flattenedPoint = cv::Point2f(newX, newY);
        }
        flattenedPoints.push_back(flattenedPoint);
    }

    return flattenedPoints;
}

// Función para aplanar los puntos 3D en un plano 2D entre x y z
std::vector<FlattenedPoint> flattenPointsXZ(const std::vector<cv::Point3f> &points)
{
    std::vector<FlattenedPoint> flattenedPoints;

    // Encontrar el punto medio más alto en y
    cv::Point3f medianPoint = findMedianPointY(points);

    // Proyectar los puntos en el plano 2D (x, z) y ajustar según y
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto &pt = points[i];
        FlattenedPoint flattenedPoint;
        flattenedPoint.originalPoint = pt;
        flattenedPoint.index = i;

        if (pt.y == medianPoint.y)
        {
            flattenedPoint.flattenedPoint = cv::Point2f(pt.x, pt.z); // Sin ajuste si y es igual
        }
        else
        {
            double scale = (medianPoint.y - pt.y) / (medianPoint.y - pt.y); // Escalar según la diferencia en y
            double newX = pt.x + (pt.x - medianPoint.x) * scale;
            double newZ = pt.z + (pt.z - medianPoint.z) * scale;
            flattenedPoint.flattenedPoint = cv::Point2f(newX, newZ);
        }

        flattenedPoints.push_back(flattenedPoint);
    }

    return flattenedPoints;
}

// Función para aplanar los puntos 3D en un plano 2D entre y y z
std::vector<FlattenedPoint> flattenPointsYZ(const std::vector<cv::Point3f> &points)
{
    std::vector<FlattenedPoint> flattenedPoints;

    // Encontrar el punto medio más alto en x
    cv::Point3f medianPoint = findMedianPointX(points);

    // Proyectar los puntos en el plano 2D (y, z) y ajustar según x
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto &pt = points[i];
        FlattenedPoint flattenedPoint;
        flattenedPoint.originalPoint = pt;
        flattenedPoint.index = i;

        if (pt.x == medianPoint.x)
        {
            flattenedPoint.flattenedPoint = cv::Point2f(pt.y, pt.z); // Sin ajuste si x es igual
        }
        else
        {
            double scale = (medianPoint.x - pt.x) / (medianPoint.x - pt.x); // Escalar según la diferencia en x
            double newY = pt.y + (pt.y - medianPoint.y) * scale;
            double newZ = pt.z + (pt.z - medianPoint.z) * scale;
            flattenedPoint.flattenedPoint = cv::Point2f(newY, newZ);
        }

        flattenedPoints.push_back(flattenedPoint);
    }

    return flattenedPoints;
}

std::vector<CaraNew> convertToCaraNew(const delaunay2d::Delaunay<float> &triangulation)
{
    std::vector<CaraNew> carasNew;

    for (const auto &triangle : triangulation.triangles)
    {
        CaraNew caraNew;

        // Mapea los puntos del triángulo a sus índices
        caraNew.indices.push_back(triangle.p0.index); // Índice del primer punto
        caraNew.indices.push_back(triangle.p1.index); // Índice del segundo punto
        caraNew.indices.push_back(triangle.p2.index); // Índice del tercer punto

        carasNew.push_back(caraNew);
    }

    return carasNew;
}

void loadAndAddModel(string modelPath, float scaleFactor, float reductionFactor, vector<ObjectProjection> &objects, float max = 1.0, float dx = 0, float dy = 0, float dz = 0.15f)
{
    Load3DModel model(modelPath, true, 0, 0, 0.15f, 0.05, scaleFactor);
    vector<cv::Point3f> vertices = model.getVertices();
    vector<cv::Point3f> normals = model.getNormals();
    vector<cv::Point2f> texCoords = model.getTexCoords();
    vector<Face> faces = model.getFaces();
    cout << "########################" << endl;
    cout << "N de Vertices: " << vertices.size() << endl;
    cout << "N de Normals: " << normals.size() << endl;
    cout << "N de TextCoords: " << texCoords.size() << endl;
    cout << "N de Faces: " << faces.size() << endl;
    // Solo aplicar decimation si hay vértices
    // if (!vertices.empty()) {
    //    Decimation decimation(vertices, normals, texCoords, faces);
    //    decimation.applyDecimation(0.5f); // Reducir al 50% (ajusta según sea necesario)
    //    vertices = decimation.getVertices();
    //    normals = decimation.getNormals();
    //    texCoords = decimation.getTexCoords();
    //    faces = decimation.getFaces();
    //}
    if (!vertices.empty())
    {
        KMeansReduction kmeansReduction(vertices, normals, texCoords, faces);
        kmeansReduction.applyReduction(reductionFactor); // Reducir al 50% (ajusta según sea necesario)
        vertices = kmeansReduction.getVertices();
        normals = kmeansReduction.getNormals();
        texCoords = kmeansReduction.getTexCoords();
        faces = kmeansReduction.getFaces();
    }
    cout << "---------------------------" << endl;
    cout << "Reduction N de Vertices: " << vertices.size() << endl;
    cout << "Reduction N de Normals: " << normals.size() << endl;
    cout << "Reduction N de TextCoords: " << texCoords.size() << endl;
    cout << "Reduction N de Faces: " << faces.size() << endl;
    /*auto flattenedPoints = flattenPointsXY(vertices);
    cout << "Flattened N de Vertices: " << flattenedPoints.size() << endl;
    /// delaunay with points2d
    std::vector<delaunay2d::Point<float>> delaunayPoints;
    for (const auto &pt : flattenedPoints)
    {
        delaunayPoints.push_back({pt.flattenedPoint.x, pt.flattenedPoint.y, pt.index});
    }
    auto triangulation = triangulate(delaunayPoints);
    cout << "Triangulation N de Triangles: " << triangulation.triangles.size() << endl;
    auto carasNew = convertToCaraNew(triangulation);
    cout << "CarasNew N de Caras: " << carasNew.size() << endl;*/
    ObjectProjection object(vertices, normals, texCoords, faces, max);
    objects.push_back(object);
}

int main()
{
    // Load 3D models
    vector<ObjectProjection> objects;
    // loadAndAddModel("models/wolf.obj", 0.0004f, 0.5f, objects, 0.15f);
    // loadAndAddModel("models/rat.obj", 0.004f, 0.5f, objects, 0.1f);
    loadAndAddModel("models/botella4mesh.obj", 0.002f, 0.5f, objects, 0.9f);
    // loadAndAddModel("models/botella.obj", 0.001f, 0.1f, objects, 0.9f);
    // loadAndAddModel("models/botella.obj", 0.001f, 0.1f, objects, 0.9f);
    /// loadAndAddModel("models/botella.obj", 0.003f, 0.1f, objects, 0.9f);
    // loadAndAddModel("models/sphere1000.obj", 0.002f, objects, 0.9f);
    // loadAndAddModel("models/sphere10000.obj", 0.002f, 0.5f, objects, 0.2f);
    // loadAndAddModel("models/woody-toy-story/source/woody.obj", 0.15f, objects);

    // Load camera
    LoadCamera camera(objects);

    if (!camera.openCamera(2))
    {
        cout << "Camera is not opened" << endl;
        return -1;
    }

    camera.showCamera();

    return 0;
}
