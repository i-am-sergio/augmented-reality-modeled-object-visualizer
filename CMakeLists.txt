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

# Copy folder models to build directory
file(COPY models DESTINATION ${CMAKE_BINARY_DIR})

# Generate executable for ARProject
add_executable(ARProject ${ARPROJECT_SOURCES})

# Generate executable for GenerateMarkers
add_executable(GenerateMarkers ${GENERATE_SOURCES})

# Link libraries
target_link_libraries(ARProject ${OpenCV_LIBS})
# Link libraries to GenerateMarkers
target_link_libraries(GenerateMarkers ${OpenCV_LIBS})
