cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING "Final installation location.")
set(CMAKE_BUILD_TYPE_INIT "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/functions.cmake")

project(pointcloudToMESH VERSION 1.0.0)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

# packages
message("***********************")
message("PCL PACKAGE")
message("***********************")

find_package(PCL 1.8 REQUIRED QUIET)

if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directory: ${PCL_DIR}")
else()
    message(FATAL_ERROR "   ERROR: PCL minimum required version 1.8. Not found")
endif()

fetch_project(
    NAME argparse
    URL https://github.com/p-ranav/argparse/archive/v2.6.tar.gz
)

fetch_project(
    NAME cloudparse
    URL https://github.com/danielTobon43/cloudparse/archive/v0.2.1.tar.gz
)

# sources
set(MAIN_SOURCE "poisson.cpp")

set(MESH_SOURCES
    "ply2obj.cpp"
)

# executables
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})

add_executable(Mesh ${MESH_SOURCES})

# headers
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_include_directories(Mesh PRIVATE include)

# libraries linking
target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES} argparse cloudparse)
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp -mno-avx -mno-avx2)

target_link_libraries(Mesh PRIVATE ${PCL_LIBRARIES} argparse cloudparse)
target_include_directories(Mesh PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(Mesh PRIVATE -Wno-cpp -mno-avx -mno-avx2)

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS Mesh
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)



# cmake_minimum_required(VERSION 3.10)
# project(ARProject)

# # C++ standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# # Find OpenCV
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

# # Add header files
# include_directories(include)

# # Add source files for Mesh
# set(MESH_SOURCES
#     mesh2.cpp
# )

# # Generate executable for Mesh
# add_executable(mesh1 ${MESH_SOURCES})

# # Link libraries to Mesh
# target_link_libraries(mesh1 ${OpenCV_LIBS})




# FOR sdlSample.cpp

# # Buscar las librerías SDL2 y OpenCV
# find_package(SDL2 REQUIRED)
# # Añadir los directorios de las cabeceras
# include_directories(${SDL2_INCLUDE_DIRS})

# # Añadir el ejecutable
# add_executable(sdlsample sdlSample.cpp)

# # Enlazar las librerías SDL2 y OpenCV
# target_link_libraries(sdlsample ${SDL2_LIBRARIES} ${OpenCV_LIBS})
