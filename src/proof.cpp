#include <GL/glut.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <limits>
#include <algorithm> // Para std::max
// Estructura para almacenar un punto 3D
struct Point3D {
    float x, y, z;
};

// Vector para almacenar todos los puntos leídos del archivo .obj
std::vector<Point3D> points;

// Variables para almacenar los límites del modelo
float minX, minY, minZ;
float maxX, maxY, maxZ;

void loadObj(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error al abrir el archivo " << filename << std::endl;
        return;
    }
    
    std::string line;
    // Inicializamos los límites con valores extremos
    minX = minY = minZ = std::numeric_limits<float>::max();
    maxX = maxY = maxZ = std::numeric_limits<float>::lowest();
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        if (prefix == "v") {
            Point3D point;
            iss >> point.x >> point.y >> point.z;
            points.push_back(point);
            // Actualizamos los límites del modelo
            if (point.x < minX) minX = point.x;
            if (point.y < minY) minY = point.y;
            if (point.z < minZ) minZ = point.z;
            if (point.x > maxX) maxX = point.x;
            if (point.y > maxY) maxY = point.y;
            if (point.z > maxZ) maxZ = point.z;
        }
    }
}

void printPoints() {
    for (const auto& point : points) {
        std::cout << "Punto: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glColor3f(0.0f, 0.0f, 0.0f); // Color de los puntos (negro)
    glPointSize(5.0f); // Tamaño de los puntos
    glBegin(GL_POINTS);
    for (const auto& point : points) {
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();

    glFlush();
}

void init() {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Color de fondo blanco
    glEnable(GL_DEPTH_TEST); // Habilitar prueba de profundidad
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.1, 100.0); // Configuración de la perspectiva
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // Posicionamos la cámara para ver todos los puntos
    float centerX = (minX + maxX) / 2.0f;
    float centerY = (minY + maxY) / 2.0f;
    float centerZ = (minZ + maxZ) / 2.0f;
    float maxDistance = std::max({maxX - minX, maxY - minY, maxZ - minZ});
    float distance = maxDistance * 1.5f; // Ajuste para que todos los puntos sean visibles
    gluLookAt(centerX, centerY, centerZ + distance, // Posición de la cámara
              centerX, centerY, centerZ, // Punto al que mira la cámara
              0.0, 1.0, 0.0); // Vector arriba
}

int main(int argc, char** argv) {
    std::string filename = "models/rat.obj";  // Especifica aquí la ruta al archivo .obj

    loadObj(filename);
    printPoints();  // Imprime todos los puntos leídos

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("Puntos 3D con OpenGL");
    init();
    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}
