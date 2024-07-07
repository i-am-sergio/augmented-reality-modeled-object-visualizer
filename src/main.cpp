#include <iostream>
#include <opencv2/opencv.hpp>
#include "LoadCamera.hpp"
#include "Load3DModel.hpp"
#include "ObjectProjection.hpp"

using namespace std;

void loadAndAddModel(string modelPath, float scaleFactor, vector<ObjectProjection>& objects, float dx = 0, float dy = 0, float dz = 0.15f) {
    Load3DModel model(modelPath, true, 0, 0, 0.15f, 0.05, scaleFactor);
    vector<cv::Point3f> vertices = model.getVertices();
    vector<cv::Point3f> normals = model.getNormals();
    vector<cv::Point2f> texCoords = model.getTexCoords();
    vector<Face> faces = model.getFaces();
    ObjectProjection object(vertices, normals, texCoords, faces);
    objects.push_back(object);
}

int main(){
    // Load 3D models
    vector<ObjectProjection> objects;
    loadAndAddModel("models/wolf.obj", 0.0004f, objects);
    loadAndAddModel("models/rat.obj", 0.004f, objects);
    loadAndAddModel("models/fox.obj", 0.002f, objects);
    loadAndAddModel("models/pegasus.obj", 0.05f, objects, 0, 0, 0);

    // Load camera
    LoadCamera camera(objects);

    if (!camera.openCamera(0)){
        cout << "Camera is not opened" << endl;
        return -1;
    }

    camera.showCamera();

    return 0;
}