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

# # Add source files for ARProject
set(ARPROJECT_SOURCES 
    src/main.cpp
)

# Add source files for GenerateMarkers
set(GENERATE_SOURCES
    src/generate.cpp
)

# Add source files for Mesh
set(MESH_SOURCES
    src/mesh.cpp
)

# Copy folder models to build directory
file(COPY models DESTINATION ${CMAKE_BINARY_DIR})

# Generate executable for ARProject
add_executable(ARProject ${ARPROJECT_SOURCES})

# Generate executable for GenerateMarkers
add_executable(GenerateMarkers ${GENERATE_SOURCES})

# Generate executable for Mesh
add_executable(Mesh ${MESH_SOURCES})

# Link libraries
target_link_libraries(ARProject ${OpenCV_LIBS})
# Link libraries to GenerateMarkers
target_link_libraries(GenerateMarkers ${OpenCV_LIBS})
# Link libraries to Mesh
target_link_libraries(Mesh ${OpenCV_LIBS})