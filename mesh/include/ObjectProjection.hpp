#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <GL/glut.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <iostream>

using namespace cv;
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_3<K> Delaunay;
typedef Delaunay::Point CGALPoint;

class ObjectProjection
{
private:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texCoords;
    std::vector<Face> faces;
    Delaunay delaunay;
    std::vector<std::pair<CGALPoint, CGALPoint>> valid_edges;
    cv::Vec3d rotation;
    cv::Vec3d translation;
    double scale;
    double movement;
    double zoom;

public:
    ObjectProjection(const std::vector<cv::Point3f> &vertices, const std::vector<cv::Point3f> &normals, const std::vector<cv::Point2f> &texCoords, const std::vector<Face> &faces)
        : vertices(vertices), normals(normals), texCoords(texCoords), faces(faces), zoom(1.0),
          rotation(cv::Vec3d(0, 0, 0)),
          translation(cv::Vec3d(0, 0, 0)), scale(1.0), movement(0.0)
    {
        buildDelaunayTriangulation();
    }

    void setTransform(const cv::Vec3d &rot, const cv::Vec3d &trans, double s, double m)
    {
        rotation = rot;
        translation = trans;
        scale = s;
        movement = m;
    }

    void setZoom(double newZoom)
    {
        zoom = newZoom;
    }

    void buildDelaunayTriangulation()
    {
        delaunay.clear();
        for (const auto &vertex : vertices)
        {
            delaunay.insert(CGALPoint(vertex.x, vertex.y, vertex.z));
        }

        std::cout << "Triangulación de Delaunay construida con " << delaunay.number_of_vertices() << " vértices." << std::endl;

        // Limpiar las aristas válidas
        valid_edges.clear();

        // Umbral de proximidad para la conexión de puntos
        const double proximity_threshold = 0.4; // Ajusta este valor según sea necesario

        // Crear un mapa de aristas para verificar si se deben agregar
        std::map<std::pair<CGALPoint, CGALPoint>, bool> edge_map;

        // Verificar algunos detalles de la triangulación
        for (auto edge = delaunay.finite_edges_begin(); edge != delaunay.finite_edges_end(); ++edge)
        {
            auto v1 = edge->first->vertex(edge->second);
            auto v2 = edge->first->vertex(edge->third);

            if (v1 != nullptr && v2 != nullptr)
            {
                CGALPoint p1 = v1->point();
                CGALPoint p2 = v2->point();
                double distance = std::sqrt(std::pow(p1.x() - p2.x(), 2) +
                                            std::pow(p1.y() - p2.y(), 2) +
                                            std::pow(p1.z() - p2.z(), 2));

                // Solo considerar aristas dentro del umbral de proximidad
                if (distance < proximity_threshold)
                {
                    // Ordenar los puntos para evitar duplicados en el mapa
                    if (p1 < p2)
                    {
                        edge_map[std::make_pair(p1, p2)] = true;
                    }
                    else
                    {
                        edge_map[std::make_pair(p2, p1)] = true;
                    }
                }
            }
        }

        // Añadir las aristas válidas a la lista
        for (const auto &entry : edge_map)
        {
            valid_edges.push_back(entry.first);
        }

        std::cout << "Número de aristas válidas: " << valid_edges.size() << std::endl;
    }

    void setupCamera()
    {
        int viewportWidth = 1000;  // Ancho de la ventana
        int viewportHeight = 1000; // Altura de la ventana

        glViewport(0, 0, viewportWidth, viewportHeight);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        // Ajustar el campo de visión (FOV), aspect ratio, y los planos de recorte cerca y lejos
        double aspectRatio = static_cast<double>(viewportWidth) / static_cast<double>(viewportHeight);
        gluPerspective(60.0, aspectRatio, 0.1, 1000.0); // Ángulo de visión de 60 grados

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Configurar la posición inicial de la cámara para que mire 180 grados a la derecha
        double cameraX = zoom;
        double cameraY = 10.0;
        double cameraZ = -13.18; // Ajusta la posición inicial en Z según la escala y posición del objeto
        double centerX = 0.0;    // Posición X del objeto detrás de la cámara
        double centerY = 0.0;    // Posición Y del objeto detrás de la cámara
        double centerZ = 0.0;    // Posición Z del objeto detrás de la cámara
        double upX = 0.0;
        double upY = 1.0;
        double upZ = 0.0;

        gluLookAt(cameraX, cameraY, cameraZ, centerX, centerY, centerZ,
                  upX, upY, upZ);
    }

    void drawObject()
    {
        setupCamera();

        glPushMatrix();

        // Aplicar la transformación de escala
        glScaled(scale, scale, scale);

        // Aplicar movimiento en Z
        glTranslated(translation[0], translation[1], translation[2]);

        // Aplicar rotaciones
        glRotated(rotation[0] * 180.0 / CV_PI, 1.0, 0.0, 0.0); // Rotación en X
        glRotated(rotation[1] * 180.0 / CV_PI, 0.0, 1.0, 0.0); // Rotación en Y
        glRotated(rotation[2] * 180.0 / CV_PI, 0.0, 0.0, 1.0); // Rotación en Z

        // Configuración de materiales para las caras del objeto
        GLfloat mat_diffuse[] = {1.0, 1.0, 1.0, 1.0}; // Color blanco
        glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

        // Dibujar las aristas válidas
        glBegin(GL_LINES);
        glColor3f(1.0f, 1.0f, 1.0f); // Set line color to white
        for (const auto &edge : valid_edges)
        {
            const auto &p1 = edge.first;
            const auto &p2 = edge.second;
            glVertex3f(p1.x(), p1.y(), p1.z());
            glVertex3f(p2.x(), p2.y(), p2.z());
        }
        glEnd();

        glPopMatrix();
        glFlush(); // Asegúrate de que todos los comandos OpenGL se hayan ejecutado
    }
};
