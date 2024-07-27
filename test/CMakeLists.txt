cmake_minimum_required(VERSION 3.10)
project(ARProject)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Add header files
include_directories(include)

# Add source files for Mesh
set(MESH_SOURCES
    mesh2.cpp
)

# Generate executable for Mesh
add_executable(mesh1 ${MESH_SOURCES})

# Link libraries to Mesh
target_link_libraries(mesh1 ${OpenCV_LIBS})






# # Buscar las librerías SDL2 y OpenCV
# find_package(SDL2 REQUIRED)
# # Añadir los directorios de las cabeceras
# include_directories(${SDL2_INCLUDE_DIRS})

# # Añadir el ejecutable
# add_executable(sdlsample sdlSample.cpp)

# # Enlazar las librerías SDL2 y OpenCV
# target_link_libraries(sdlsample ${SDL2_LIBRARIES} ${OpenCV_LIBS})
