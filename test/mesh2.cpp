#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "Load3DModel.hpp"
#include "DelaunayTet.hpp"
using namespace std;

// Función para convertir cv::Point3f a Point3D
template <typename T>
delaunay::Point3D<T> convertPoint(const cv::Point3f& p)
{
    return delaunay::Point3D<T>(p.x, p.y, p.z);
}


int main() {
    // Load 3D model
    Load3DModel model("models/fox.obj", true, 0, 0, 0.15f);
    vector<cv::Point3f> vertices = model.getVertices();

    cout << "########### DELAUNAY 3D #############" << endl;

    // Convertir los puntos de cv::Point3f a delaunay::Point3D<float>
    std::vector<delaunay::Point3D<float>> delaunayPoints;
    for (const auto& cvPoint : vertices)
    {
        delaunayPoints.push_back(convertPoint<float>(cvPoint));
    }

    // Realizar la triangulación de Delaunay en 3D
    delaunay::Delaunay3D<float> delaunayResult = delaunay::triangulate<float>(delaunayPoints);

    // Archivo para guardar tetraedros
    std::ofstream outFileTet("outputTet.txt");

    // Escribir tetraedros en el archivo
    for (const auto& tet : delaunayResult.tetrahedra)
    {
        outFileTet << "Tetrahedron: "
                  << "p0: " << tet.p0 << ", "
                  << "p1: " << tet.p1 << ", "
                  << "p2: " << tet.p2 << ", "
                  << "p3: " << tet.p3 << std::endl;
    }

    // Archivo para guardar aristas
    std::ofstream outFileEdge("outputEdge.txt");

    // Escribir aristas en el archivo
    for (const auto& edge : delaunayResult.edges)
    {
        outFileEdge << "Edge: " << edge << std::endl;
    }



    // Archivo para escribir el resultado en formato .obj
    std::ofstream outFile("output.obj");

    // Escribir vértices en el archivo .obj
    for (const auto& vertex : vertices) {
        outFile << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
    }

    
    
    return 0;
}