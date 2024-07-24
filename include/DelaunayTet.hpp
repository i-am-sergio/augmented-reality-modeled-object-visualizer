#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

namespace delaunay {

constexpr double eps = 1e-4;

template <typename T>
struct Point3D {
  T x, y, z;

  Point3D() : x{0}, y{0}, z{0} {}
  Point3D(T _x, T _y, T _z) : x{_x}, y{_y}, z{_z} {}

  template <typename U>
  Point3D(U _x, U _y, U _z) : x{static_cast<T>(_x)}, y{static_cast<T>(_y)}, z{static_cast<T>(_z)}
  {
  }

  friend std::ostream& operator<<(std::ostream& os, const Point3D<T>& p)
  {
    os << "x=" << p.x << "  y=" << p.y << "  z=" << p.z;
    return os;
  }

  bool operator==(const Point3D<T>& other) const
  {
    return (other.x == x && other.y == y && other.z == z);
  }

  bool operator!=(const Point3D<T>& other) const { return !operator==(other); }
};

template <typename T>
struct Edge3D {
  using Node = Point3D<T>;
  Node p0, p1;

  Edge3D(Node const& _p0, Node const& _p1) : p0{_p0}, p1{_p1} {}

  friend std::ostream& operator<<(std::ostream& os, const Edge3D& e)
  {
    os << "p0: [" << e.p0 << " ] p1: [" << e.p1 << "]";
    return os;
  }

  bool operator==(const Edge3D& other) const
  {
    return ((other.p0 == p0 && other.p1 == p1) ||
            (other.p0 == p1 && other.p1 == p0));
  }
};

template <typename T>
struct Tetrahedron {
  using Node = Point3D<T>;
  Node p0, p1, p2, p3;

  struct Circumsphere {
    T x, y, z, radius2;
  } circumsphere;

  Tetrahedron(const Node& _p0, const Node& _p1, const Node& _p2, const Node& _p3)
      : p0{_p0}, p1{_p1}, p2{_p2}, p3{_p3}
  {
    calculateCircumsphere();
  }

  void calculateCircumsphere()
  {
    auto ax = p1.x - p0.x;
    auto ay = p1.y - p0.y;
    auto az = p1.z - p0.z;
    auto bx = p2.x - p0.x;
    auto by = p2.y - p0.y;
    auto bz = p2.z - p0.z;
    auto cx = p3.x - p0.x;
    auto cy = p3.y - p0.y;
    auto cz = p3.z - p0.z;

    auto D = 2 * (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx));
    auto p1Sq = p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
    auto p2Sq = p2.x * p2.x + p2.y * p2.y + p2.z * p2.z;
    auto p3Sq = p3.x * p3.x + p3.y * p3.y + p3.z * p3.z;
    auto p0Sq = p0.x * p0.x + p0.y * p0.y + p0.z * p0.z;

    circumsphere.x = ((p1Sq - p0Sq) * (by * cz - bz * cy) + (p2Sq - p0Sq) * (ay * cz - az * cy) + (p3Sq - p0Sq) * (ay * bz - az * by)) / D;
    circumsphere.y = ((p1Sq - p0Sq) * (bx * cz - bz * cx) + (p2Sq - p0Sq) * (ax * cz - az * cx) + (p3Sq - p0Sq) * (ax * bz - az * bx)) / D;
    circumsphere.z = ((p1Sq - p0Sq) * (bx * cy - by * cx) + (p2Sq - p0Sq) * (ax * cy - ay * cx) + (p3Sq - p0Sq) * (ax * by - ay * bx)) / D;

    auto dx = circumsphere.x - p0.x;
    auto dy = circumsphere.y - p0.y;
    auto dz = circumsphere.z - p0.z;
    circumsphere.radius2 = dx * dx + dy * dy + dz * dz;
  }

  bool isPointInsideCircumsphere(const Node& p) const
  {
    auto dx = circumsphere.x - p.x;
    auto dy = circumsphere.y - p.y;
    auto dz = circumsphere.z - p.z;
    auto dist2 = dx * dx + dy * dy + dz * dz;
    return dist2 <= circumsphere.radius2 + eps;
  }
};

template <typename T>
struct Delaunay3D {
  std::vector<Tetrahedron<T>> tetrahedra;
  std::vector<Edge3D<T>> edges;
};

template <
    typename T,
    typename = typename std::enable_if<std::is_floating_point<T>::value>::type>
Delaunay3D<T> triangulate(const std::vector<Point3D<T>>& points)
{
  using Node = Point3D<T>;
  if (points.size() < 4) {
    return Delaunay3D<T>{};
  }

  // Compute a bounding super tetrahedron
  T xmin = points[0].x, xmax = points[0].x;
  T ymin = points[0].y, ymax = points[0].y;
  T zmin = points[0].z, zmax = points[0].z;
  for (const auto& p : points) {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
    zmin = std::min(zmin, p.z);
    zmax = std::max(zmax, p.z);
  }

  T dx = xmax - xmin;
  T dy = ymax - ymin;
  T dz = zmax - zmin;
  T maxd = std::max({dx, dy, dz});
  T midx = (xmin + xmax) / static_cast<T>(2);
  T midy = (ymin + ymax) / static_cast<T>(2);
  T midz = (zmin + zmax) / static_cast<T>(2);

  Delaunay3D<T> d;
  Node p0{midx, midy - 20 * maxd, midz - 20 * maxd};
  Node p1{midx - 20 * maxd, midy + 20 * maxd, midz + 20 * maxd};
  Node p2{midx + 20 * maxd, midy + 20 * maxd, midz - 20 * maxd};
  Node p3{midx, midy - 20 * maxd, midz + 20 * maxd};

  d.tetrahedra.emplace_back(p0, p1, p2, p3);

  for (const auto& p : points) {
    std::vector<Edge3D<T>> edges;
    std::vector<Tetrahedron<T>> tmps;

    std::cout<<"p = "<<p<<std::endl;

    for (const auto& tet : d.tetrahedra) {
      if (tet.isPointInsideCircumsphere(p)) {
        edges.emplace_back(tet.p0, tet.p1);
        edges.emplace_back(tet.p0, tet.p2);
        edges.emplace_back(tet.p0, tet.p3);
        edges.emplace_back(tet.p1, tet.p2);
        edges.emplace_back(tet.p1, tet.p3);
        edges.emplace_back(tet.p2, tet.p3);
      } else {
        tmps.push_back(tet);
      }
    }

    d.tetrahedra = tmps;

    std::vector<Edge3D<T>> unique_edges;
    for (const auto& e : edges) {
      auto it = std::find_if(unique_edges.begin(), unique_edges.end(),
                             [&e](const Edge3D<T>& other) {
                               return other == e;
                             });
      if (it == unique_edges.end()) {
        unique_edges.push_back(e);
      }
    }

    for (const auto& e : unique_edges) {
      d.tetrahedra.emplace_back(e.p0, e.p1, e.p1, p);
    }
  }

  d.tetrahedra.erase(
      std::remove_if(d.tetrahedra.begin(), d.tetrahedra.end(),
                     [p0, p1, p2, p3](const Tetrahedron<T>& tet) {
                       return (tet.p0 == p0 || tet.p0 == p1 || tet.p0 == p2 ||
                               tet.p0 == p3 || tet.p1 == p0 || tet.p1 == p1 ||
                               tet.p1 == p2 || tet.p1 == p3 || tet.p2 == p0 ||
                               tet.p2 == p1 || tet.p2 == p2 || tet.p2 == p3 ||
                               tet.p3 == p0 || tet.p3 == p1 || tet.p3 == p2 ||
                               tet.p3 == p3);
                     }),
      d.tetrahedra.end());

  return d;
}

}  // namespace delaunay
