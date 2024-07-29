#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include "Load3DModel.hpp"
#include "LoadCamera.hpp"
#include "ObjectProjection.hpp"
#include "KMeansReduction.hpp"
using namespace std;

void loadAndAddModel(string modelPath, bool swapyz,float scaleFactor, float reductionFactor, vector<ObjectProjection> &objects, float max = 1.0, float dx = 0, float dy = 0, float dz = 0.15f)
{
    Load3DModel model(modelPath, swapyz, 0, 0, 0.15f, 0.05, scaleFactor);
    vector<cv::Point3f> vertices = model.getVertices();
    vector<cv::Point3f> normals = model.getNormals();
    vector<cv::Point2f> texCoords = model.getTexCoords();
    vector<Face> faces = model.getFaces();
    cout << "====================================================" << endl;
    cout << "---------- MODEL: " << modelPath << " ----------" << endl;
    cout << "- Nro de Vertices: " << vertices.size() << endl;
    if (!vertices.empty()) // Solo aplicar si hay vértices
    {
        KMeansReduction kmeansReduction(vertices, normals, texCoords, faces);
        kmeansReduction.applyReduction(reductionFactor); // Reducir al 50% (ajusta según sea necesario)
        vertices = kmeansReduction.getVertices();
        normals = kmeansReduction.getNormals();
        texCoords = kmeansReduction.getTexCoords();
        faces = kmeansReduction.getFaces();
    }
    cout << "---------- REDUCTION TO " << reductionFactor * 100 << "% ----------" << endl;
    cout << "- Post-Reduction Nro de Vertices: " << vertices.size() << endl;
    ObjectProjection object(vertices, normals, texCoords, faces, max, 3.1, 0.1);
    objects.push_back(object);
    cout << "====================================================" << endl;
}

int main()
{
    // Load 3D models
    vector<ObjectProjection> objects;
    loadAndAddModel("models/botellapoissonmesh.obj", false, 0.003f, 0.3f, objects, 0.9f); // botella con poisson mesh generado por src/ply2obj.cpp
    loadAndAddModel("models/wolf.obj", true, 0.0005f, 0.9f, objects, 0.15f); // funciona
    loadAndAddModel("models/nescafe.obj", true, 1.8f, 0.15f, objects, 0.15f, 0.0f,0.0f,-1.0f); // funciona
    loadAndAddModel("models/smartphone.obj", true, 0.1f, 0.2f, objects, 0.15f); // funciona
    loadAndAddModel("models/pegasus.obj", true, 0.1f, 0.05f, objects, 0.15f); // funciona
    loadAndAddModel("models/rat.obj", true, 0.007f, 0.5f, objects, 0.1f); // funciona
    loadAndAddModel("models/fox.obj", true, 0.004f, 1.0f, objects, 0.15f); // funciona
    loadAndAddModel("models/sheet.obj", true, 0.1f, 1.0f, objects, 0.15f); // funciona
    // loadAndAddModel("models/chess_pawn.obj", false, 0.15f, 0.05f, objects, 0.15f); // funciona
    // loadAndAddModel("models/chess_knight.obj", false, 0.06f, 0.008f, objects, 2.0f); // funciona
    // loadAndAddModel("models/botellawithmesh.obj", 0.003f, 0.3f, objects, 0.9f); // funciona
    // loadAndAddModel("models/botella4mesh.obj", 0.002f, 0.5f, objects, 0.9f);

    // loadAndAddModel("models/botella.obj", 0.001f, 0.1f, objects, 0.9f);
    // loadAndAddModel("models/botella.obj", 0.003f, 0.1f, objects, 0.9f);
    // loadAndAddModel("models/sphere1000.obj", 0.002f, objects, 0.9f);
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
