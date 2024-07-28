cmake_minimum_required(VERSION 3.20)
project(ARProject)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define the source directories
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Add header files directories
include_directories(${INCLUDE_DIR}/interface)
include_directories(${INCLUDE_DIR}/implement)

# Gather all .cpp files from the implement directory
file(GLOB IMPLEMENT_SOURCES "${INCLUDE_DIR}/implement/*.cpp")

# Add source files for ARProject
set(ARPROJECT_SOURCES
    ${SOURCE_DIR}/main.cpp
    ${IMPLEMENT_SOURCES}
)

# Add source files for GenerateMarkers
set(GENERATE_SOURCES
    ${SOURCE_DIR}/generate.cpp
    ${IMPLEMENT_SOURCES}
)

# Add source files for Mesh
set(MESH_SOURCES
    ${SOURCE_DIR}/mesh.cpp
    ${IMPLEMENT_SOURCES}
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
target_link_libraries(GenerateMarkers ${OpenCV_LIBS})
target_link_libraries(Mesh ${OpenCV_LIBS})
