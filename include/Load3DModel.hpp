#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct Vec3 {
    float x, y, z;
};

struct Vec2 {
    float u, v;
};

struct Face {
    std::vector<int> vertices;
    std::vector<int> texCoords;
    std::vector<int> normals;

    // operator << overloading
    friend std::ostream& operator<<(std::ostream& os, const Face& face){
        // vertices
        os << "[";
        if (face.vertices.size() != 0){
            for (int i = 0; i < face.vertices.size(); i++){
                os << face.vertices[i] << " ";
            }
        }
        os << "],";
        // normals
        os << "[";
        if (face.normals.size() != 0){
            for (int i = 0; i < face.normals.size(); i++){
                os << face.normals[i] << " ";
            }
        }
        os << "],";
        // texCoords
        os << "[";
        if (face.texCoords.size() != 0){
            for (int i = 0; i < face.texCoords.size(); i++){
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
        std::vector<Vec3> vertices;
        std::vector<Vec3> normals;
        std::vector<Vec2> texCoords;
        std::vector<Face> faces;

        // Desplazamiento
        float dx, dy, dz;
        // Escala
        float scaleFactor;
        // Marker length
        float markerLength;

    public: 
        Load3DModel(const std::string& filename, bool swapyz = true, float dx = 0, float dy = 0, float dz = 0, float markerLength = 0.1, float scaleFactor = 1.0){
            this->dx = dx;
            this->dy = dy;
            this->dz = dz;
            this->scaleFactor = scaleFactor;
            this->markerLength = markerLength;
            load(filename, swapyz);
        }

        void load(const std::string& filename, bool swapyz){
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Unable to open file: " << filename << std::endl;
                return;
            }

            std::string line;
            while (std::getline(file, line)) {
                if (line.substr(0, 1) == "#") continue;

                std::istringstream ss(line); // Convert string to stream
                std::string type; // v, vn, vt, f
                ss >> type; // Read the first word

                if (type == "v") {
                    Vec3 vertex;
                    ss >> vertex.x >> vertex.y >> vertex.z;
                    if (swapyz) {
                        std::swap(vertex.y, vertex.z);
                    }
                    vertex.x += dx;
                    vertex.y += dy;
                    vertex.z += dz;
                    vertices.push_back(vertex);
                } else if (type == "vn") {
                    Vec3 normal;
                    ss >> normal.x >> normal.y >> normal.z;
                    if (swapyz) {
                        std::swap(normal.y, normal.z);
                    }
                    normals.push_back(normal);

                } else if (type == "vt") {
                    Vec2 texcoord;
                    ss >> texcoord.u >> texcoord.v;
                    texCoords.push_back(texcoord);
                
                } else if (type == "f") {
                    Face face;
                    std::string vertexData;
                    while (ss >> vertexData) {
                        std::istringstream vss(vertexData);
                        std::string index;
                        int i = 0;
                        while (std::getline(vss, index, '/')) {
                            int idx = index.empty() ? 0 : std::stoi(index);
                            if (i == 0) {
                                face.vertices.push_back(idx);
                            } else if (i == 1) {
                                face.texCoords.push_back(idx);
                            } else if (i == 2) {
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

        std::vector<Vec3> getVertices(){ return vertices; }

        std::vector<Vec3> getNormals(){ return normals; }

        std::vector<Vec2> getTexCoords(){ return texCoords; }

        std::vector<Face> getFaces(){ return faces; }

        void printVertices(){
            for (auto& vertex : vertices){
                std::cout << "v = (" << vertex.x << " " << vertex.y << " " << vertex.z << ")" << std::endl;
            }
            std::cout << "# vertices: " << vertices.size() << std::endl;
        }

        void printNormals(){
            for (auto& normal : normals){
                std::cout << "vn = (" << normal.x << " " << normal.y << " " << normal.z << ")" << std::endl;
            }
            std::cout << "# normals: " << normals.size() << std::endl;
        }

        void printTexCoords(){
            for (auto& texcoord : texCoords){
                std::cout << "vt = (" << texcoord.u << " " << texcoord.v << ")" << std::endl;
            }
            std::cout << "# texCoords: " << texCoords.size() << std::endl;
        }

        void printFaces(){
            for (auto& face : faces){
                std::cout << "f = ";
                std::cout << face << std::endl;
            }
            std::cout << "# faces: " << faces.size() << std::endl;
        }

        void scaleVertices(float scaleFactor){
            for (auto& vertex : vertices){
                vertex.x *= scaleFactor;
                vertex.y *= scaleFactor;
                vertex.z *= scaleFactor;
            }

            if (normals.size() != 0){
                for (auto& normal : normals){
                    normal.x *= scaleFactor;
                    normal.y *= scaleFactor;
                    normal.z *= scaleFactor;
                }
            }

            if (texCoords.size() != 0){
                for (auto& texcoord : texCoords){
                    texcoord.u *= scaleFactor;
                    texcoord.v *= scaleFactor;
                }
            }
        }
};