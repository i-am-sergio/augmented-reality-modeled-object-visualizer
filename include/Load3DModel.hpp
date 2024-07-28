#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct Face
{
    std::vector<int> vertices;
    std::vector<int> texCoords;
    std::vector<int> normals;

    // Sobrecarga del operador <<
    friend std::ostream &operator<<(std::ostream &os, const Face &face)
    {
        // vertices
        os << "[";
        if (!face.vertices.empty())
        {
            for (size_t i = 0; i < face.vertices.size(); ++i)
            {
                os << face.vertices[i] << " ";
            }
        }
        os << "],";
        // normals
        os << "[";
        if (!face.normals.empty())
        {
            for (size_t i = 0; i < face.normals.size(); ++i)
            {
                os << face.normals[i] << " ";
            }
        }
        os << "],";
        // texCoords
        os << "[";
        if (!face.texCoords.empty())
        {
            for (size_t i = 0; i < face.texCoords.size(); ++i)
            {
                os << face.texCoords[i] << " ";
            }
        }
        os << "]";
        return os;
    }
};

class Load3DModel
{
private:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texCoords;
    std::vector<Face> faces;

    // Desplazamiento
    float dx, dy, dz;
    // Escala
    float scaleFactor;
    // Longitud del marcador
    float markerLength;

public:
    Load3DModel(const std::string &filename, bool swapyz = false, float dx = 0, float dy = 0, float dz = 0, float markerLength = 0.1f, float scaleFactor = 1.0f)
    {
        this->dx = dx;
        this->dy = dy;
        this->dz = dz;
        this->scaleFactor = scaleFactor;
        this->markerLength = markerLength;
        load(filename, swapyz);
    }

    void load(const std::string &filename, bool swapyz)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Unable to open file: " << filename << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line.substr(0, 1) == "#")
                continue;

            std::istringstream ss(line); // Convertir string a stream
            std::string type;            // v, vn, vt, f
            ss >> type;                  // Leer la primera palabra

            if (type == "v")
            {
                cv::Point3f vertex;
                ss >> vertex.x >> vertex.y >> vertex.z;
                if (swapyz)
                {
                    std::swap(vertex.y, vertex.z);
                }
                vertex.x += dx;
                vertex.y += dy;
                vertex.z += dz;
                vertices.push_back(vertex);
            }
            else if (type == "vn")
            {
                cv::Point3f normal;
                ss >> normal.x >> normal.y >> normal.z;
                if (swapyz)
                {
                    std::swap(normal.y, normal.z);
                }
                normals.push_back(normal);
            }
            else if (type == "vt")
            {
                cv::Point2f texcoord;
                ss >> texcoord.x >> texcoord.y;
                texCoords.push_back(texcoord);
            }
            else if (type == "f")
            {
                Face face;
                std::string vertexData;
                while (ss >> vertexData)
                {
                    std::istringstream vss(vertexData);
                    std::string index;
                    int i = 0;
                    while (std::getline(vss, index, '/'))
                    {
                        int idx = index.empty() ? 0 : std::stoi(index);
                        if (i == 0)
                        {
                            face.vertices.push_back(idx);
                        }
                        else if (i == 1)
                        {
                            face.texCoords.push_back(idx);
                        }
                        else if (i == 2)
                        {
                            face.normals.push_back(idx);
                        }
                        ++i;
                    }
                }
                faces.push_back(face);
            }
        }
        scaleVertices(scaleFactor);
        file.close();
    }

    std::vector<cv::Point3f> getVertices() { return vertices; }

    std::vector<cv::Point3f> getNormals() { return normals; }

    std::vector<cv::Point2f> getTexCoords() { return texCoords; }

    std::vector<Face> getFaces() { return faces; }

    void printVertices()
    {
        for (auto &vertex : vertices)
        {
            std::cout << "v = (" << vertex.x << " " << vertex.y << " " << vertex.z << ")" << std::endl;
        }
        std::cout << "# vertices: " << vertices.size() << std::endl;
    }

    void printNormals()
    {
        for (auto &normal : normals)
        {
            std::cout << "vn = (" << normal.x << " " << normal.y << " " << normal.z << ")" << std::endl;
        }
        std::cout << "# normals: " << normals.size() << std::endl;
    }

    void printTexCoords()
    {
        for (auto &texcoord : texCoords)
        {
            std::cout << "vt = (" << texcoord.x << " " << texcoord.y << ")" << std::endl;
        }
        std::cout << "# texCoords: " << texCoords.size() << std::endl;
    }

    void printFaces()
    {
        for (auto &face : faces)
        {
            std::cout << "f = ";
            std::cout << face << std::endl;
        }
        std::cout << "# faces: " << faces.size() << std::endl;
    }

    void scaleVertices(float scaleFactor)
    {
        for (auto &vertex : vertices)
        {
            vertex.x *= scaleFactor;
            vertex.y *= scaleFactor;
            vertex.z *= scaleFactor;
        }

        if (!normals.empty())
        {
            for (auto &normal : normals)
            {
                normal.x *= scaleFactor;
                normal.y *= scaleFactor;
                normal.z *= scaleFactor;
            }
        }

        if (!texCoords.empty())
        {
            for (auto &texcoord : texCoords)
            {
                texcoord.x *= scaleFactor;
                texcoord.y *= scaleFactor;
            }
        }
    }
};
