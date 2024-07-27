#pragma once

#include <iostream>
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include "Load3DModel.hpp"
#include "LoadCamera.hpp"
#include "ObjectProjection.hpp"
#include "Decimation.hpp"
#include "Delaunay2D.hpp"
#include "KMeansReduction.hpp"
#include "FlattenedPoint.hpp"

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
