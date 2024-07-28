#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

// Estructura para almacenar información de una cara
struct Face
{
    std::vector<int> vertices;
    std::vector<int> texCoords;
    std::vector<int> normals;

    // Sobrecarga del operador <<
    friend std::ostream &operator<<(std::ostream &os, const Face &face);
};

// Clase para cargar y manipular un modelo 3D
class Load3DModel
{
private:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texCoords;
    std::vector<Face> faces;
    float dx, dy, dz;   // Desplazamiento
    float scaleFactor;  // Escala
    float markerLength; // Longitud del marcador

public:
    Load3DModel(const std::string &filename, bool swapyz = false, float dx = 0, float dy = 0, float dz = 0, float markerLength = 0.1f, float scaleFactor = 1.0f);

    void load(const std::string &filename, bool swapyz);

    std::vector<cv::Point3f> getVertices();
    std::vector<cv::Point3f> getNormals();
    std::vector<cv::Point2f> getTexCoords();
    std::vector<Face> getFaces();

    void printVertices();
    void printNormals();
    void printTexCoords();
    void printFaces();

    void scaleVertices(float scaleFactor);
};
