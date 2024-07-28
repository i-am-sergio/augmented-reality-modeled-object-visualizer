#include "Load3DModel.hpp"

// Sobrecarga del operador <<
std::ostream &operator<<(std::ostream &os, const Face &face)
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

// Constructor de la clase Load3DModel
Load3DModel::Load3DModel(const std::string &filename, bool swapyz, float dx, float dy, float dz, float markerLength, float scaleFactor)
    : dx(dx), dy(dy), dz(dz), scaleFactor(scaleFactor), markerLength(markerLength)
{
    load(filename, swapyz);
}

// Método para cargar el modelo 3D desde un archivo
void Load3DModel::load(const std::string &filename, bool swapyz)
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

// Métodos para obtener los datos del modelo
std::vector<cv::Point3f> Load3DModel::getVertices()
{
    return vertices;
}

std::vector<cv::Point3f> Load3DModel::getNormals()
{
    return normals;
}

std::vector<cv::Point2f> Load3DModel::getTexCoords()
{
    return texCoords;
}

std::vector<Face> Load3DModel::getFaces()
{
    return faces;
}

// Métodos para imprimir los datos del modelo
void Load3DModel::printVertices()
{
    for (auto &vertex : vertices)
    {
        std::cout << "v = (" << vertex.x << " " << vertex.y << " " << vertex.z << ")" << std::endl;
    }
    std::cout << "# vertices: " << vertices.size() << std::endl;
}

void Load3DModel::printNormals()
{
    for (auto &normal : normals)
    {
        std::cout << "vn = (" << normal.x << " " << normal.y << " " << normal.z << ")" << std::endl;
    }
    std::cout << "# normals: " << normals.size() << std::endl;
}

void Load3DModel::printTexCoords()
{
    for (auto &texcoord : texCoords)
    {
        std::cout << "vt = (" << texcoord.x << " " << texcoord.y << ")" << std::endl;
    }
    std::cout << "# texCoords: " << texCoords.size() << std::endl;
}

void Load3DModel::printFaces()
{
    for (auto &face : faces)
    {
        std::cout << "f = ";
        std::cout << face << std::endl;
    }
    std::cout << "# faces: " << faces.size() << std::endl;
}

// Método para escalar los vértices
void Load3DModel::scaleVertices(float scaleFactor)
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
