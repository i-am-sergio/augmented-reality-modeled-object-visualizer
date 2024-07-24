#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <bits/stdc++.h>

namespace delaunay {

constexpr double eps = 1e-4;

template <typename T>
struct Point3 {
    T x, y, z;

    Point3() : x{0}, y{0}, z{0} {}
    Point3(T _x, T _y, T _z) : x{_x}, y{_y}, z{_z} {}

    template <typename U>
    Point3(U _x, U _y, U _z) : x{static_cast<T>(_x)}, y{static_cast<T>(_y)}, z{static_cast<T>(_z)} {}

    friend std::ostream& operator<<(std::ostream& os, const Point3<T>& p) {
        os << "x=" << p.x << "  y=" << p.y << "  z=" << p.z;
        return os;
    }

    bool operator==(const Point3<T>& other) const {
        return (other.x == x && other.y == y && other.z == z);
    }

    bool operator!=(const Point3<T>& other) const {
        return !operator==(other);
    }
};

template <typename T>
struct Tetrahedron {
    Point3<T> p0, p1, p2, p3;
    Point3<T> circ_center;
    T circ_radius;

    Tetrahedron(const Point3<T>& _p0, const Point3<T>& _p1, const Point3<T>& _p2, const Point3<T>& _p3)
        : p0{_p0}, p1{_p1}, p2{_p2}, p3{_p3} {}

    // Calculate the circumscribing sphere of the tetrahedron
    void calculateCircumsphere() {
        T A[4][4] = {
            {p0.x, p0.y, p0.z, 1},
            {p1.x, p1.y, p1.z, 1},
            {p2.x, p2.y, p2.z, 1},
            {p3.x, p3.y, p3.z, 1}
        };
        T B[4] = {
            p0.x * p0.x + p0.y * p0.y + p0.z * p0.z,
            p1.x * p1.x + p1.y * p1.y + p1.z * p1.z,
            p2.x * p2.x + p2.y * p2.y + p2.z * p2.z,
            p3.x * p3.x + p3.y * p3.y + p3.z * p3.z
        };

        T det = 0;
        T cx = 0, cy = 0, cz = 0, r = 0;

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                if (i == j) {
                    det += A[i][j] * (A[(i+1)%4][(j+1)%4] * A[(i+2)%4][(j+2)%4] * A[(i+3)%4][(j+3)%4] -
                                      A[(i+1)%4][(j+1)%4] * A[(i+2)%4][(j+3)%4] * A[(i+3)%4][(j+2)%4] +
                                      A[(i+2)%4][(j+1)%4] * A[(i+3)%4][(j+2)%4] * A[(i+1)%4][(j+3)%4] -
                                      A[(i+2)%4][(j+1)%4] * A[(i+3)%4][(j+1)%4] * A[(i+1)%4][(j+2)%4]);
                }
            }
        }

        if (std::fabs(det) < eps) {
            return;
        }

        cx = (B[0] * (A[1][1] * A[2][2] * A[3][3] - A[1][1] * A[2][3] * A[3][2] +
                       A[2][1] * A[3][2] * A[1][3] - A[2][1] * A[3][3] * A[1][2] +
                       A[3][1] * A[1][2] * A[2][3] - A[3][1] * A[1][3] * A[2][2]) -
             B[1] * (A[0][1] * A[2][2] * A[3][3] - A[0][1] * A[2][3] * A[3][2] +
                     A[2][1] * A[3][2] * A[0][3] - A[2][1] * A[3][3] * A[0][2] +
                     A[3][1] * A[0][2] * A[2][3] - A[3][1] * A[0][3] * A[2][2]) +
             B[2] * (A[0][1] * A[1][2] * A[3][3] - A[0][1] * A[1][3] * A[3][2] +
                     A[1][1] * A[3][2] * A[0][3] - A[1][1] * A[3][3] * A[0][2] +
                     A[3][1] * A[0][2] * A[1][3] - A[3][1] * A[0][3] * A[1][2]) -
             B[3] * (A[0][1] * A[1][2] * A[2][3] - A[0][1] * A[1][3] * A[2][2] +
                     A[1][1] * A[2][2] * A[0][3] - A[1][1] * A[2][3] * A[0][2] +
                     A[2][1] * A[0][2] * A[1][3] - A[2][1] * A[0][3] * A[1][2]);

        cy = (B[0] * (A[1][0] * A[2][2] * A[3][3] - A[1][0] * A[2][3] * A[3][2] +
                       A[2][0] * A[3][2] * A[1][3] - A[2][0] * A[3][3] * A[1][2] +
                       A[3][0] * A[1][2] * A[2][3] - A[3][0] * A[1][3] * A[2][2]) -
             B[1] * (A[0][0] * A[2][2] * A[3][3] - A[0][0] * A[2][3] * A[3][2] +
                     A[2][0] * A[3][2] * A[0][3] - A[2][0] * A[3][3] * A[0][2] +
                     A[3][0] * A[0][2] * A[2][3] - A[3][0] * A[0][3] * A[2][2]) +
             B[2] * (A[0][0] * A[1][2] * A[3][3] - A[0][0] * A[1][3] * A[3][2] +
                     A[1][0] * A[3][2] * A[0][3] - A[1][0] * A[3][3] * A[0][2] +
                     A[3][0] * A[0][2] * A[1][3] - A[3][0] * A[0][3] * A[1][2]) -
             B[3] * (A[0][0] * A[1][2] * A[2][3] - A[0][0] * A[1][3] * A[2][2] +
                     A[1][0] * A[2][2] * A[0][3] - A[1][0] * A[2][3] * A[0][2] +
                     A[2][0] * A[0][2] * A[1][3] - A[2][0] * A[0][3] * A[1][2]);

        cz = (B[0] * (A[1][0] * A[2][1] * A[3][3] - A[1][0] * A[2][3] * A[3][1] +
                       A[2][0] * A[3][1] * A[1][3] - A[2][0] * A[3][3] * A[1][1] +
                       A[3][0] * A[1][1] * A[2][3] - A[3][0] * A[1][3] * A[2][1]) -
             B[1] * (A[0][0] * A[2][1] * A[3][3] - A[0][0] * A[2][3] * A[3][1] +
                     A[2][0] * A[3][1] * A[0][3] - A[2][0] * A[3][3] * A[0][1] +
                     A[3][0] * A[0][1] * A[2][3] - A[3][0] * A[0][3] * A[2][1]) +
             B[2] * (A[0][0] * A[1][1] * A[3][3] - A[0][0] * A[1][3] * A[3][1] +
                     A[1][0] * A[3][1] * A[0][3] - A[1][0] * A[3][3] * A[0][1] +
                     A[3][0] * A[0][1] * A[1][3] - A[3][0] * A[0][3] * A[1][1]) -
             B[3] * (A[0][0] * A[1][1] * A[2][3] - A[0][0] * A[1][3] * A[2][1] +
                     A[1][0] * A[2][1] * A[0][3] - A[1][0] * A[2][3] * A[0][1] +
                     A[2][0] * A[0][1] * A[1][3] - A[2][0] * A[0][3] * A[1][1]);

        r = std::sqrt(cx * cx + cy * cy + cz * cz);

        circ_center = {cx / (2 * det), cy / (2 * det), cz / (2 * det)};
        circ_radius = r / (2 * det);
    }
};

template <typename T>
struct Triangle3D {
    Point3<T> p0, p1, p2;

    Triangle3D(const Point3<T>& _p0, const Point3<T>& _p1, const Point3<T>& _p2)
        : p0{_p0}, p1{_p1}, p2{_p2} {}
};

template <typename T>
struct ConvexHull {
    std::vector<Triangle3D<T>> triangles;
};

template <typename T, typename = typename std::enable_if<std::is_floating_point<T>::value>::type>
ConvexHull<T> triangulate3D(const std::vector<Point3<T>>& points) {
    using Point = Point3<T>;

    if (points.size() < 4) {
        return ConvexHull<T>{};
    }

    // Find bounding box and create a super tetrahedron
    auto xmin = points[0].x, xmax = xmin;
    auto ymin = points[0].y, ymax = ymin;
    auto zmin = points[0].z, zmax = zmin;

    for (const auto& pt : points) {
        xmin = std::min(xmin, pt.x);
        xmax = std::max(xmax, pt.x);
        ymin = std::min(ymin, pt.y);
        ymax = std::max(ymax, pt.y);
        zmin = std::min(zmin, pt.z);
        zmax = std::max(zmax, pt.z);
    }

    const auto dx = xmax - xmin;
    const auto dy = ymax - ymin;
    const auto dz = zmax - zmin;
    const auto dmax = std::max({dx, dy, dz});
    const auto midx = (xmin + xmax) / static_cast<T>(2.);
    const auto midy = (ymin + ymax) / static_cast<T>(2.);
    const auto midz = (zmin + zmax) / static_cast<T>(2.);

    // Create a super tetrahedron that contains all points
    Point p0{midx - 20 * dmax, midy - 20 * dmax, midz - 20 * dmax};
    Point p1{midx + 20 * dmax, midy + 20 * dmax, midz - 20 * dmax};
    Point p2{midx - 20 * dmax, midy + 20 * dmax, midz + 20 * dmax};
    Point p3{midx + 20 * dmax, midy - 20 * dmax, midz + 20 * dmax};

    std::vector<Tetrahedron<T>> tetrahedra;
    tetrahedra.emplace_back(Tetrahedron<T>{p0, p1, p2, p3});

    for (const auto& pt : points) {
        std::vector<Tetrahedron<T>> newTetrahedra;

        for (const auto& tet : tetrahedra) {
            tet.calculateCircumsphere();
            T dist = std::sqrt(std::pow(pt.x - tet.circ_center.x, 2) +
                               std::pow(pt.y - tet.circ_center.y, 2) +
                               std::pow(pt.z - tet.circ_center.z, 2));

            if (dist < tet.circ_radius) {
                newTetrahedra.emplace_back(Tetrahedron<T>{tet.p0, tet.p1, tet.p2, pt});
                newTetrahedra.emplace_back(Tetrahedron<T>{tet.p0, tet.p1, tet.p3, pt});
                newTetrahedra.emplace_back(Tetrahedron<T>{tet.p0, tet.p2, tet.p3, pt});
                newTetrahedra.emplace_back(Tetrahedron<T>{tet.p1, tet.p2, tet.p3, pt});
            }
        }

        tetrahedra = std::move(newTetrahedra);
    }

    // Remove tetrahedra that have vertices of the super tetrahedron
    tetrahedra.erase(
        std::remove_if(tetrahedra.begin(), tetrahedra.end(),
                       [&](const Tetrahedron<T>& tet) {
                           return (tet.p0 == p0 || tet.p1 == p0 || tet.p2 == p0 || tet.p3 == p0 ||
                                   tet.p0 == p1 || tet.p1 == p1 || tet.p2 == p1 || tet.p3 == p1 ||
                                   tet.p0 == p2 || tet.p1 == p2 || tet.p2 == p2 || tet.p3 == p2 ||
                                   tet.p0 == p3 || tet.p1 == p3 || tet.p2 == p3 || tet.p3 == p3);
                       }),
        tetrahedra.end());

    // Extract the surface triangles from the tetrahedra
    ConvexHull<T> hull;

    std::unordered_map<std::tuple<Point, Point, Point>, bool> triangle_map;

    for (const auto& tet : tetrahedra) {
        hull.triangles.emplace_back(Triangle3D<T>{tet.p0, tet.p1, tet.p2});
        hull.triangles.emplace_back(Triangle3D<T>{tet.p0, tet.p1, tet.p3});
        hull.triangles.emplace_back(Triangle3D<T>{tet.p0, tet.p2, tet.p3});
        hull.triangles.emplace_back(Triangle3D<T>{tet.p1, tet.p2, tet.p3});
    }

    // Remove duplicate triangles
    auto is_duplicate = [&](const Triangle3D<T>& t) {
        auto pts = std::vector<Point>{t.p0, t.p1, t.p2};
        std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
            return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
        });
        return triangle_map.find(std::make_tuple(pts[0], pts[1], pts[2])) != triangle_map.end();
    };

    hull.triangles.erase(
        std::remove_if(hull.triangles.begin(), hull.triangles.end(), is_duplicate),
        hull.triangles.end());

    return hull;
}

} // namespace delaunay
