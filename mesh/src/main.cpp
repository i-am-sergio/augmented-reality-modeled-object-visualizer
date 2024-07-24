#include <iostream>
#include <opencv2/opencv.hpp>
#include <GLFW/glfw3.h>
#include "Load3DModel.hpp"
#include "ObjectProjection.hpp"

using namespace std;

// Global variables
ObjectProjection *objectProjection = nullptr;
float rotationX = 0.0f;
float rotationY = 0.0f;
float zoom = -45.0f; // Initial zoom level
int lastX = 0, lastY = 0;
bool isDragging = false;
float sensitivity = 0.01f; // Mouse movement sensitivity

// Set up object transformations
void setupTransformations()
{
    cv::Vec3d rotation(rotationX, rotationY, 0.0);
    cv::Vec3d translation(0, 0, 0);
    double scale = 1.0;
    double movement = 0.0;

    if (objectProjection)
    {
        objectProjection->setTransform(rotation, translation, scale, movement);
    }
}

// GLFW display function
void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Adjust view based on zoom level
    gluLookAt(0.0, 0.0, zoom, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    if (objectProjection)
    {
        objectProjection->drawObject();
    }

    glfwSwapBuffers(glfwGetCurrentContext());
}

// OpenGL setup
void initOpenGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glClearColor(0.0, 0.0, 0.0, 1.0);

    // Set perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1000.0 / 1000.0, 1.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

// Load and add the 3D model
void loadAndAddModel(const string &modelPath, float scaleFactor, float dx = 0, float dy = 0, float dz = 0.15f)
{
    Load3DModel model(modelPath, true, dx, dy, dz, 0.05, scaleFactor);
    vector<cv::Point3f> vertices = model.getVertices();
    vector<cv::Point3f> normals = model.getNormals();
    vector<cv::Point2f> texCoords = model.getTexCoords();
    vector<Face> faces = model.getFaces();

    objectProjection = new ObjectProjection(vertices, normals, texCoords, faces);
}

// Mouse callback for rotation
void mouseCallback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            double x, y;
            cout << x << " " << y << endl;
            glfwGetCursorPos(window, &x, &y);
            lastX = static_cast<int>(x);
            lastY = static_cast<int>(y);
            isDragging = true;
        }
        else if (action == GLFW_RELEASE)
        {
            isDragging = false;
        }
    }
}

// Cursor position callback for rotation
void cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
    if (isDragging)
    {
        float deltaX = (xpos - lastX) * sensitivity;
        float deltaY = (ypos - lastY) * sensitivity;

        rotationX += deltaX;
        rotationY -= deltaY;

        lastX = static_cast<int>(xpos);
        lastY = static_cast<int>(ypos);

        setupTransformations();
        display();
    }
}

// Keyboard callback for zoom
void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        if (key == GLFW_KEY_UP)
        {
            zoom -= (sensitivity + 1) * 2.0f; // Zoom in
            cout << "Zoom: " << zoom << endl;
            objectProjection->setZoom(zoom); // Actualiza la cámara con el nuevo zoom
            display();                       // Redibuja la escena con el nuevo zoom
        }
        else if (key == GLFW_KEY_DOWN)
        {
            zoom += (sensitivity + 1) * 2.0f; // Zoom out
            cout << "Zoom: " << zoom << endl;
            objectProjection->setZoom(zoom); // Actualiza la cámara con el nuevo zoom
            display();                       // Redibuja la escena con el nuevo zoom
        }
    }
}

int main(int argc, char **argv)
{
    if (!glfwInit())
    {
        cerr << "Failed to initialize GLFW" << endl;
        return -1;
    }

    GLFWwindow *window = glfwCreateWindow(800, 600, "3D Model with Delaunay Triangulation", nullptr, nullptr);
    if (!window)
    {
        cerr << "Failed to create GLFW window" << endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    // Initialize OpenGL
    initOpenGL();

    // Load and add the 3D model
    loadAndAddModel("models/chess_pawn.obj", 1.0f);

    // Set initial transformations
    setupTransformations();

    // Set callbacks
    glfwSetMouseButtonCallback(window, mouseCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetKeyCallback(window, keyCallback);

    // Main rendering loop
    while (!glfwWindowShouldClose(window))
    {
        display();
        glfwPollEvents();
    }

    delete objectProjection;

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
