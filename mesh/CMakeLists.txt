cmake_minimum_required(VERSION 3.20)
project(TriangulationProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to Release for better performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find GMP
find_package(GMP REQUIRED)
include_directories(${GMP_INCLUDE_DIRS})

# Find CGAL
find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDE_DIRS})

# Find OpenGL, GLEW, GLFW, and GLUT
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)



if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 not found. Please install Eigen3 or adjust the path in CMakeLists.txt")
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# Include directories
include_directories(
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

# Source files
file(GLOB_RECURSE SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    # Uncomment and adjust if you have additional source files
    # "${CMAKE_SOURCE_DIR}/src/Components/LoadModel/*.cpp"
    # "${CMAKE_SOURCE_DIR}/src/Components/DelaunayTriangulation/*.cpp"
)

# Header files
file(GLOB_RECURSE HEADERS
    "${CMAKE_SOURCE_DIR}/include/*.hpp"
)

# Add executable
add_executable(TriangulationProject ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(TriangulationProject
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    glfw
    ${GLUT_LIBRARIES} # Add GLUT libraries here
    ${CGAL_LIBRARIES}
    ${CGAL_3RD_PARTY_LIBRARIES}
)

# Create target directory for model files
add_custom_command(
    TARGET TriangulationProject PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_BINARY_DIR}/models
)

# Copy .obj files from models to the build directory
file(GLOB MODEL_FILES "${CMAKE_SOURCE_DIR}/models/*.obj")
foreach(MODEL_FILE ${MODEL_FILES})
    add_custom_command(
        TARGET TriangulationProject POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${MODEL_FILE}
        ${CMAKE_BINARY_DIR}/models/
    )
endforeach()
