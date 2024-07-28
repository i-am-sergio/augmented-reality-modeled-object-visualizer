#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <iostream>
#include <fstream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file.ply> <output_file.obj>" << std::endl;
        return -1;
    }

    std::string input_filename = argv[1];
    std::string output_filename = argv[2];

    // Definir el tipo de punto (en este caso XYZ)
    pcl::PolygonMesh mesh;

    // Leer el archivo .ply
    if (pcl::io::loadPLYFile(input_filename, mesh) == -1) {
        PCL_ERROR("Couldn't read file %s \n", input_filename.c_str());
        return -1;
    }

    // Abrir archivo de salida
    std::ofstream outFile(output_filename);
    if (!outFile) {
        std::cerr << "Could not open the output file: " << output_filename << std::endl;
        return -1;
    }

    // Escribir los vértices en formato .obj
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    for (const auto& point : cloud.points) {
        outFile << "v " << point.x << " " << point.y << " " << point.z << std::endl;
    }

    // Escribir las caras en formato .obj
    for (const auto& polygon : mesh.polygons) {
        outFile << "f ";
        for (const auto& vertex : polygon.vertices) {
            // OBJ files are 1-indexed
            outFile << vertex + 1 << " ";
        }
        outFile << std::endl;
    }

    outFile.close();
    std::cout << "Mesh data has been written to " << output_filename << std::endl;

    return 0;
}
