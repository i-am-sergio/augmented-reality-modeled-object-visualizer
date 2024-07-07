#include <iostream>
#include <opencv2/opencv.hpp>
#include "LoadCamera.hpp"
#include "Load3DModel.hpp"
#include "ObjectProjection.hpp"

int main(){
    
    Load3DModel obj("models/wolf.obj", true, 0, 0, 0.15f, 0.05, 0.0002f);
    obj.printVertices();

    LoadCamera camera;
    if (!camera.openCamera(0)){
        std::cout << "Camera is not opened" << std::endl;
        return -1;
    }

    // ObjectProjection objProjection(obj.getVertices(), obj.getNormals(), obj.getTexCoords(), obj.getFaces());
    camera.showCamera();

    return 0;
}