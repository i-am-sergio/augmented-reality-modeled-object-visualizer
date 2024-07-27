cmake_minimum_required(VERSION 3.10)
project(SDL_OpenCV_Example)

# Buscar las librerías SDL2 y OpenCV
find_package(SDL2 REQUIRED)
find_package(OpenCV REQUIRED)

# Añadir los directorios de las cabeceras
include_directories(${SDL2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Añadir el ejecutable
add_executable(sdl_opencv_example testcpp.cpp)

# Enlazar las librerías SDL2 y OpenCV
target_link_libraries(sdl_opencv_example ${SDL2_LIBRARIES} ${OpenCV_LIBS})
