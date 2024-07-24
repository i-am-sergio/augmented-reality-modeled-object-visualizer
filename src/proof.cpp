#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <limits>
#include <stdexcept>

template <typename T>
std::vector<delaunay::Point3<T>> readObjVertices(const std::string& filename) {
    std::vector<delaunay::Point3<T>> vertices;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file.");
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            T x, y, z;
            if (!(iss >> x >> y >> z)) {
                throw std::runtime_error("Error reading vertex data.");
            }
            vertices.emplace_back(x, y, z);
        }
    }

    file.close();
    return vertices;
}
