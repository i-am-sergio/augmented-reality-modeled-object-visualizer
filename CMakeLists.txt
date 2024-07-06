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

# Add source files
set(SOURCES 
    src/main.cpp
)

# Copy folder models to build directory
file(COPY models DESTINATION ${CMAKE_BINARY_DIR})

# Generate executable
add_executable(ARProject ${SOURCES})

# Link libraries
target_link_libraries(ARProject ${OpenCV_LIBS})
