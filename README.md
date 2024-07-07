# Augmented Reality Modeled Object Visualizer

![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)

Augmented Reality Modeled Object Visualizer is a simple augmented reality application developed using C++, CMake, and OpenCV. The project identifies a aruco pattern using the camera and places a 3D object (.obj) on that pattern.


## Proyect Structure

```sh
augmented-reality-modeled-object-visualizer/
├── CMakeLists.txt
├── README.md
├── .gitignore
│
├── models/
│   └── model.obj
├── include/
│   └── ARProject.hpp
│   └── ModelLoader.hpp
│   └── PatternDetector.hpp
├── src/
│   └── main.cpp
│   └── ARProject.cpp
│   └── ModelLoader.cpp
│   └── PatternDetector.cpp
├── build/
│   └── (CMake files generated)
├── assets/
│   └── shaders/
│       └── vertex_shader.glsl
│       └── fragment_shader.glsl
└── tests/
    └── main.py
    └── 3d.py
    └── test_main.cpp
```

## Compile application

```bash
mkdir build
cd build
cmake ..
cmake --build .
./ARProject
```

## License:

This project is licensed under [Creative Commons Atribución-NoComercial-CompartirIgual 4.0 Internacional](http://creativecommons.org/licenses/by-nc-sa/4.0/):

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">
  <img alt="Licencia Creative Commons" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" />
</a>