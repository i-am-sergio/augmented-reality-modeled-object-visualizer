#pragma once

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>
#include <tuple>

namespace delaunay3d {

constexpr double eps = 1e-10;

template <typename T>
struct Point {
    T x, y, z;

    Point() : x(0), y(0), z(0) {}
    Point(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

    bool operator==(const Point& other) const {
        return (std::abs(x - other.x) < eps && std::abs(y - other.y) < eps && std::abs(z - other.z) < eps);
    }

    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
};

template <typename T>
struct Edge {
    Point<T> p0, p1;

    Edge(Point<T> _p0, Point<T> _p1) : p0(_p0), p1(_p1) {
        if (p1 < p0) std::swap(p0, p1);
    }

    bool operator==(const Edge& other) const {
        return (p0 == other.p0 && p1 == other.p1);
    }

    bool operator<(const Edge& other) const {
        return std::tie(p0, p1) < std::tie(other.p0, other.p1);
    }
};

template <typename T>
struct Tetrahedron {
    Point<T> p0, p1, p2, p3;

    Tetrahedron(Point<T> _p0, Point<T> _p1, Point<T> _p2, Point<T> _p3)
        : p0(_p0), p1(_p1), p2(_p2), p3(_p3) {}

    // Add any necessary methods for tetrahedron manipulation
};

template <typename T>
struct Delaunay3D {
    std::vector<Tetrahedron<T>> tetrahedrons;

    Delaunay3D(const std::vector<Point<T>>& points) {
        // Placeholder for initial super tetrahedron
        Point<T> p0(0, 0, 0), p1(1000, 0, 0), p2(0, 1000, 0), p3(0, 0, 1000);
        Tetrahedron<T> super(p0, p1, p2, p3);
        tetrahedrons.push_back(super);

        for (const auto& point : points) {
            std::vector<Edge<T>> boundary_edges;
            std::vector<Tetrahedron<T>> new_tetrahedrons;

            auto it = tetrahedrons.begin();
            while (it != tetrahedrons.end()) {
                if (isPointInsideCircumsphere(point, *it)) {
                    collectBoundaryEdges(*it, boundary_edges);
                    it = tetrahedrons.erase(it);
                } else {
                    ++it;
                }
            }

            removeDuplicateEdges(boundary_edges);

            for (const auto& edge : boundary_edges) {
                new_tetrahedrons.emplace_back(edge.p0, edge.p1, point, p3); // p3 is a placeholder
            }

            tetrahedrons.insert(tetrahedrons.end(), new_tetrahedrons.begin(), new_tetrahedrons.end());
        }

        // Remove tetrahedrons connected to the initial super tetrahedron vertices
        removeSuperTetrahedronVertices(p0, p1, p2, p3);
    }

private:
    bool isPointInsideCircumsphere(const Point<T>& pt, const Tetrahedron<T>& tetra) {
        auto det = [](const Point<T>& p1, const Point<T>& p2, const Point<T>& p3, const Point<T>& p4) {
            return (p1.x - p4.x) * ((p2.y - p4.y) * (p3.z - p4.z) - (p2.z - p4.z) * (p3.y - p4.y))
                 - (p1.y - p4.y) * ((p2.x - p4.x) * (p3.z - p4.z) - (p2.z - p4.z) * (p3.x - p4.x))
                 + (p1.z - p4.z) * ((p2.x - p4.x) * (p3.y - p4.y) - (p2.y - p4.y) * (p3.x - p4.x));
        };
    
        T mat[4][4] = {
            {tetra.p0.x, tetra.p0.y, tetra.p0.z, 1},
            {tetra.p1.x, tetra.p1.y, tetra.p1.z, 1},
            {tetra.p2.x, tetra.p2.y, tetra.p2.z, 1},
            {tetra.p3.x, tetra.p3.y, tetra.p3.z, 1}
        };
    
        T d = det(tetra.p0, tetra.p1, tetra.p2, tetra.p3);
        if (std::abs(d) < eps) return false; // Degenerate tetrahedron
    
        for (int i = 0; i < 4; ++i) {
            mat[i][0] = pt.x;
            mat[i][1] = pt.y;
            mat[i][2] = pt.z;
            if (i > 0) {
                mat[i-1][0] = tetra.p0.x;
                mat[i-1][1] = tetra.p0.y;
                mat[i-1][2] = tetra.p0.z;
            }
            T dp = det(
                {mat[0][0], mat[0][1], mat[0][2]},
                {mat[1][0], mat[1][1], mat[1][2]},
                {mat[2][0], mat[2][1], mat[2][2]},
                {mat[3][0], mat[3][1], mat[3][2]}
            );
            if ((d < 0 && dp > 0) || (d > 0 && dp < 0)) return false;
        }
    
        return true;
    }

    void collectBoundaryEdges(const Tetrahedron<T>& tetra, std::vector<Edge<T>>& edges) {
        edges.push_back(Edge<T>(tetra.p0, tetra.p1));
        edges.push_back(Edge<T>(tetra.p1, tetra.p2));
        edges.push_back(Edge<T>(tetra.p2, tetra.p0));
        edges.push_back(Edge<T>(tetra.p0, tetra.p3));
        edges.push_back(Edge<T>(tetra.p1, tetra.p3));
        edges.push_back(Edge<T>(tetra.p2, tetra.p3));
    }

    void removeDuplicateEdges(std::vector<Edge<T>>& edges) {
        std::sort(edges.begin(), edges.end());
        edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    }

    void removeSuperTetrahedronVertices(const Point<T>& p0, const Point<T>& p1, const Point<T>& p2, const Point<T>& p3) {
        tetrahedrons.erase(std::remove_if(tetrahedrons.begin(), tetrahedrons.end(),
            [&p0, &p1, &p2, &p3](const Tetrahedron<T>& tetra) {
                return tetra.p0 == p0 || tetra.p1 == p1 || tetra.p2 == p2 || tetra.p3 == p3;
            }), tetrahedrons.end());
    }
};

}