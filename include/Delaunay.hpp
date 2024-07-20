#pragma once // Evita la inclusión múltiple del archivo de encabezado

#include <algorithm> // Incluye funciones de algoritmo estándar como min, max, etc.
#include <iostream>  // Incluye funciones de entrada/salida estándar
#include <vector>    // Incluye la clase de plantilla std::vector

// Definición del namespace 'delaunay'
namespace delaunay
{

  // Constante de precisión
  constexpr double eps = 1e-4;

  // Estructura para representar un punto en 2D
  template <typename T>
  struct Point
  {
    // Coordenadas del punto
    T x;
    T y;

    // Constructor por defecto que inicializa el punto en (0, 0)
    Point()
    {
      x = 0;
      y = 0;
    }

    // Constructor que inicializa el punto con coordenadas dadas
    Point(T _x, T _y)
    {
      x = _x;
      y = _y;
    }

    // Constructor que permite la conversión de tipos de datos
    template <typename U>
    Point(U _x, U _y)
    {
      x = static_cast<T>(_x);
      y = static_cast<T>(_y);
    }

    // Sobrecarga del operador '<<' para imprimir el punto
    friend std::ostream &operator<<(std::ostream &os, const Point<T> &p)
    {
      os << "x=" << p.x << "  y=" << p.y;
      return os;
    }

    // Sobrecarga del operador '==' para comparar dos puntos
    bool operator==(const Point<T> &other) const
    {
      return (other.x == x && other.y == y);
    }

    // Sobrecarga del operador '!=' para comparar dos puntos
    bool operator!=(const Point<T> &other) const { return !operator==(other); }
  };


  // Estructura para representar una arista (línea entre dos puntos)
  template <typename T>
  struct Edge
  {
    using Node = Point<T>; // Define Node como un alias para Point<T>
    Node p0, p1;           // Los dos puntos que definen la arista

    // Constructor que inicializa una arista con dos puntos dados
    Edge(Node const &_p0, Node const &_p1)
    {
      p0 = _p0;
      p1 = _p1;
    }

    // Sobrecarga del operador '<<' para imprimir la arista
    friend std::ostream &operator<<(std::ostream &os, const Edge &e)
    {
      os << "p0: [" << e.p0 << " ] p1: [" << e.p1 << "]";
      return os;
    }

    // Sobrecarga del operador '==' para comparar dos aristas
    bool operator==(const Edge &other) const
    {
      return ((other.p0 == p0 && other.p1 == p1) ||
              (other.p0 == p1 && other.p1 == p0));
    }
  };


  // Estructura para representar un círculo (usado en la triangulación)
  template <typename T>
  struct Circle
  {
    T x, y, radius;     // Coordenadas del centro del círculo y su radio
    Circle() = default; // Constructor por defecto
  };

  // Estructura para representar un triángulo formado por tres puntos y tres aristas
  template <typename T>
  struct Triangle
  {
    using Node = Point<T>; // Define Node como un alias para Point<T>
    Node p0, p1, p2;       // Los tres puntos que definen el triángulo
    Edge<T> e0, e1, e2;    // Las tres aristas que definen el triángulo
    Circle<T> circle;      // El círculo circunscrito del triángulo

    // Constructor que inicializa un triángulo con tres puntos dados
    Triangle(const Node &_p0, const Node &_p1, const Node &_p2)
        : p0{_p0},
          p1{_p1},
          p2{_p2},
          e0{_p0, _p1},
          e1{_p1, _p2},
          e2{_p0, _p2},
          circle{}
    {
      // Calcula las coordenadas del círculo circunscrito del triángulo
      const auto ax = p1.x - p0.x; // Diferencia en x entre p1 y p0
      const auto ay = p1.y - p0.y; // Diferencia en y entre p1 y p0
      const auto bx = p2.x - p0.x; // Diferencia en x entre p2 y p0
      const auto by = p2.y - p0.y; // Diferencia en y entre p2 y p0

      // Calcula las coordenadas del centro del círculo circunscrito
      const auto m = p1.x * p1.x - p0.x * p0.x + p1.y * p1.y - p0.y * p0.y;
      const auto u = p2.x * p2.x - p0.x * p0.x + p2.y * p2.y - p0.y * p0.y;
      const auto s = 1. / (2. * (ax * by - ay * bx));

      circle.x = ((p2.y - p0.y) * m + (p0.y - p1.y) * u) * s; // Coordenada x del centro
      circle.y = ((p0.x - p2.x) * m + (p1.x - p0.x) * u) * s; // Coordenada y del centro

      const auto dx = p0.x - circle.x; // Diferencia en x entre p0 y el centro
      const auto dy = p0.y - circle.y; // Diferencia en y entre p0 y el centro
      circle.radius = dx * dx + dy * dy; // Calcula el radio del círculo circunscrito
    }
  };


  // Estructura que contiene la triangulación de Delaunay, con triángulos y aristas
  template <typename T>
  struct Delaunay
  {
    std::vector<Triangle<T>> triangles; // Vector de triángulos de Delaunay
    std::vector<Edge<T>> edges;         // Vector de aristas de Delaunay
  };

  // Función para realizar la triangulación de Delaunay
  template <
      typename T,
      typename = typename std::enable_if<std::is_floating_point<T>::value>::type>
  Delaunay<T> triangulate(const std::vector<Point<T>> &points)
  {
    using Node = Point<T>; // Define Node como un alias para Point<T>

    // Si hay menos de 3 puntos, no se puede hacer la triangulación
    if (points.size() < 3)
    {
      return Delaunay<T>{};
    }

    // Encuentra los límites (xmin, xmax, ymin, ymax) del conjunto de puntos
    auto xmin = points[0].x; // Inicializa los límites con el primer punto
    auto xmax = xmin; // Inicializa los límites con el primer punto
    auto ymin = points[0].y; // Inicializa los límites con el primer punto
    auto ymax = ymin; // Inicializa los límites con el primer punto
    
    // Itera sobre todos los puntos
    for (auto const &pt : points) 
    {
      xmin = std::min(xmin, pt.x); // Actualiza xmin si < punto actual
      xmax = std::max(xmax, pt.x); // Actualiza xmax si > punto actual
      ymin = std::min(ymin, pt.y); // Actualiza ymin si < punto actual
      ymax = std::max(ymax, pt.y); // Actualiza ymax si > punto actual
    }

    // Calcula el centro y la dimensión máxima del conjunto de puntos
    const auto dx = xmax - xmin; // Diferencia en x
    const auto dy = ymax - ymin; // Diferencia en y
    const auto dmax = std::max(dx, dy); // Dimensión máxima
    const auto midx = (xmin + xmax) / static_cast<T>(2.); // Centro en x
    const auto midy = (ymin + ymax) / static_cast<T>(2.); // Centro en y

    // Inicializa la triangulación con un triángulo supergrande
    auto d = Delaunay<T>{};

    const auto p0 = Node{midx - 20 * dmax, midy - dmax}; // Punto inferior izquierdo
    const auto p1 = Node{midx, midy + 20 * dmax}; // Punto superior
    const auto p2 = Node{midx + 20 * dmax, midy - dmax}; // Punto inferior derecho
    d.triangles.emplace_back(Triangle<T>{p0, p1, p2}); // Añade el triángulo supergrande

    // Añade cada punto uno a uno a la triangulación
    for (auto const &pt : points)
    {
      std::vector<Edge<T>> edges;
      std::vector<Triangle<T>> tmps;

      // Verifica si el punto está dentro del círculo circunscrito de algún triángulo existente
      for (auto const &tri : d.triangles)
      {
        const auto dist = (tri.circle.x - pt.x) * (tri.circle.x - pt.x) +
                          (tri.circle.y - pt.y) * (tri.circle.y - pt.y);
                          // Calcula la distancia al centro del círculo circunscrito
        // Si el punto está dentro del círculo circunscrito, añade las aristas del triángulo a la lista de aristas
        if ((dist - tri.circle.radius) <= eps) // Si el punto está dentro del círculo circunscrito
        {
          edges.push_back(tri.e0); // Añade la arista 0 del triángulo
          edges.push_back(tri.e1); // Añade la arista 1 del triángulo
          edges.push_back(tri.e2); // Añade la arista 2 del triángulo
        }
        else // Si el punto no está dentro del círculo circunscrito
        {
          tmps.push_back(tri); // Añade el triángulo a la lista de triángulos
        }
      }

      // Elimina las aristas duplicadas
      std::vector<bool> remove(edges.size(), false); // Vector de booleanos para marcar aristas a eliminar
      for (auto it1 = edges.begin(); it1 != edges.end(); ++it1) // Itera sobre todas las aristas
      {
        for (auto it2 = edges.begin(); it2 != edges.end(); ++it2) // Itera sobre todas las aristas
        {
          if (it1 == it2) // Si las aristas son iguales, continúa
          {
            continue;
          }
          if (*it1 == *it2) // Si las aristas son iguales, marca para eliminar
          {
            remove[std::distance(edges.begin(), it1)] = true; // Marca la arista it1 para eliminar
            remove[std::distance(edges.begin(), it2)] = true; // Marca la arista it2 para eliminar
          }
        }
      }

      edges.erase(
          std::remove_if(edges.begin(), edges.end(),
                         [&](auto const &e)
                         { return remove[&e - &edges[0]]; }), // Elimina las aristas marcadas para eliminar
          edges.end());

      // Actualiza la triangulación con los nuevos triángulos formados por el nuevo punto y las aristas restantes
      for (auto const &e : edges)
      {
        // Añade un nuevo triángulo con dos puntos de la arista y el nuevo punto
        tmps.push_back({e.p0, e.p1, pt}); 
      }

      d.triangles = std::move(tmps);
    }

    // Añade las aristas de los triángulos restantes a la lista de aristas de la triangulación
    for (auto const &tri : d.triangles)
    {
      d.edges.push_back(tri.e0); // Añade la arista 0 del triángulo
      d.edges.push_back(tri.e1); // Añade la arista 1 del triángulo
      d.edges.push_back(tri.e2); // Añade la arista 2 del triángulo
    }

    return d; // Devuelve la estructura Delaunay con los triángulos y aristas calculados
  }

}