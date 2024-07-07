#include <iostream>
#include <opencv2/opencv.hpp>
#include "LoadCamera.hpp"
#include "Load3DModel.hpp"

int main(){
    
    Load3DModel model("models/rat.obj");

    // print vertices
    model.printVertices();

    

    LoadCamera camera;
    if (!camera.openCamera(0)){
        std::cout << "Camera is not opened" << std::endl;
        return -1;
    }

    camera.showCamera();

    return 0;
}