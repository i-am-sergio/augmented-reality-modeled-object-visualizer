#include <iostream>
#include <opencv2/opencv.hpp>
#include "LoadCamera.hpp"
#include "Load3DModel.hpp"
#include "ObjectProjection.hpp"
#include "Decimation.hpp"
#include "KMeansReduction.hpp"

using namespace std;

void loadAndAddModel(string modelPath, float scaleFactor, float reductionFactor, vector<ObjectProjection> &objects, float max = 1.0, float dx = 0, float dy = 0, float dz = 0.15f)
{
    Load3DModel model(modelPath, true, 0, 0, 0.15f, 0.05, scaleFactor);
    vector<cv::Point3f> vertices = model.getVertices();
    vector<cv::Point3f> normals = model.getNormals();
    vector<cv::Point2f> texCoords = model.getTexCoords();
    vector<Face> faces = model.getFaces();
    cout << "########################" << endl;
    cout << "N de Vertices: " << vertices.size() << endl;
    cout << "N de Normals: " << normals.size() << endl;
    cout << "N de TextCoords: " << texCoords.size() << endl;
    cout << "N de Faces: " << faces.size() << endl;
    // Solo aplicar decimation si hay vértices
    // if (!vertices.empty()) {
    //    Decimation decimation(vertices, normals, texCoords, faces);
    //    decimation.applyDecimation(0.5f); // Reducir al 50% (ajusta según sea necesario)
    //    vertices = decimation.getVertices();
    //    normals = decimation.getNormals();
    //    texCoords = decimation.getTexCoords();
    //    faces = decimation.getFaces();
    //}
    if (!vertices.empty())
    {
        KMeansReduction kmeansReduction(vertices, normals, texCoords, faces);
        kmeansReduction.applyReduction(reductionFactor); // Reducir al 50% (ajusta según sea necesario)
        vertices = kmeansReduction.getVertices();
        normals = kmeansReduction.getNormals();
        texCoords = kmeansReduction.getTexCoords();
        faces = kmeansReduction.getFaces();
    }
    cout << "---------------------------" << endl;
    cout << "Reduction N de Vertices: " << vertices.size() << endl;
    cout << "Reduction N de Normals: " << normals.size() << endl;
    cout << "Reduction N de TextCoords: " << texCoords.size() << endl;
    cout << "Reduction N de Faces: " << faces.size() << endl;
    ObjectProjection object(vertices, normals, texCoords, faces, max);
    objects.push_back(object);
}

void LoadAndAddModel2(string modelPath, float scaleFactor, vector<ObjectProjection> &objects, float max = 1.0, float dx = 0, float dy = 0, float dz = 0.15f)
{
}

int main()
{
    // Load 3D models
    vector<ObjectProjection> objects;
    loadAndAddModel("models/wolf.obj", 0.0004f, 0.5f, objects, 0.15f);
    loadAndAddModel("models/rat.obj", 0.004f, 0.5f, objects, 0.1f);
    loadAndAddModel("models/Corona.obj", 0.010f, 0.5f, objects, 0.9f);
    // loadAndAddModel("models/sphere1000.obj", 0.002f, objects, 0.9f);
    loadAndAddModel("models/sphere10000.obj", 0.002f, 0.5f, objects, 0.2f);
    // loadAndAddModel("models/woody-toy-story/source/woody.obj", 0.15f, objects);

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
