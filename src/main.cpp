#include <iostream>
#include <opencv2/opencv.hpp>
#include "LoadCamera.hpp"
#include "Load3DModel.hpp"
#include "ObjectProjection.hpp"

using namespace std;

void loadAndAddModel(string modelPath, float scaleFactor, vector<ObjectProjection> &objects, float max = 1.0, float dx = 0, float dy = 0, float dz = 0.15f)
{
    Load3DModel model(modelPath, true, 0, 0, 0.15f, 0.05, scaleFactor);
    vector<cv::Point3f> vertices = model.getVertices();
    vector<cv::Point3f> normals = model.getNormals();
    vector<cv::Point2f> texCoords = model.getTexCoords();
    vector<Face> faces = model.getFaces();
    ObjectProjection object(vertices, normals, texCoords, faces, max);
    objects.push_back(object);
}

int main()
{
    // Load 3D models
    vector<ObjectProjection> objects;
    loadAndAddModel("models/wolf.obj", 0.0004f, objects, 0.15f);
    loadAndAddModel("models/rat.obj", 0.004f, objects, 0.12f);
    loadAndAddModel("models/sphere1000.obj", 0.002f, objects, 0.9f);
    loadAndAddModel("models/sphere10000.obj", 0.08f, objects, 0.9f);
    loadAndAddModel("models/woody-toy-story/source/woody.obj", 0.15f, objects);

    // Load camera
    LoadCamera camera(objects);

    if (!camera.openCamera(0))
    {
        cout << "Camera is not opened" << endl;
        return -1;
    }

    camera.showCamera();

    return 0;
}