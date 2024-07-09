#include <SDL2/SDL.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

struct Vertex {
    float x, y, z;
};

struct Cube {
    std::vector<Vertex> vertices;
    std::vector<std::pair<int, int>> edges;
};

void drawCube(SDL_Renderer* renderer, const Cube& cube, float angleX, float angleY) {
    // Create rotation matrices
    cv::Mat rotX = (cv::Mat_<float>(3, 3) << 
        1, 0, 0,
        0, cos(angleX), -sin(angleX),
        0, sin(angleX), cos(angleX));

    cv::Mat rotY = (cv::Mat_<float>(3, 3) << 
        cos(angleY), 0, sin(angleY),
        0, 1, 0,
        -sin(angleY), 0, cos(angleY));

    // Apply rotation to each vertex
    std::vector<cv::Point2f> projectedVertices;
    for (const auto& vertex : cube.vertices) {
        cv::Mat v = (cv::Mat_<float>(3, 1) << vertex.x, vertex.y, vertex.z);
        cv::Mat rotatedV = rotY * rotX * v;

        // Simple perspective projection
        float z = 5 + rotatedV.at<float>(2, 0);
        float x = rotatedV.at<float>(0, 0) / z;
        float y = rotatedV.at<float>(1, 0) / z;

        projectedVertices.emplace_back(WINDOW_WIDTH / 2 + x * 200, WINDOW_HEIGHT / 2 - y * 200);
    }

    // Draw edges
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    for (const auto& edge : cube.edges) {
        SDL_RenderDrawLine(renderer, 
                           projectedVertices[edge.first].x, projectedVertices[edge.first].y, 
                           projectedVertices[edge.second].x, projectedVertices[edge.second].y);
    }
}

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* win = SDL_CreateWindow("SDL + OpenCV Cube", 100, 100, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (win == nullptr) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == nullptr) {
        SDL_DestroyWindow(win);
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    Cube cube = {
        { // vertices
            {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
            {-1, -1,  1}, {1, -1,  1}, {1, 1,  1}, {-1, 1,  1}
        },
        { // edges
            {0, 1}, {1, 2}, {2, 3}, {3, 0},
            {4, 5}, {5, 6}, {6, 7}, {7, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        }
    };

    bool quit = false;
    SDL_Event e;
    int mouseX = 0, mouseY = 0;
    float angleX = 0.0f, angleY = 0.0f;

    while (!quit) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                quit = true;
            } else if (e.type == SDL_MOUSEMOTION) {
                int dx = e.motion.x - mouseX;
                int dy = e.motion.y - mouseY;
                angleX += dy * 0.01f;
                angleY += dx * 0.01f;
                mouseX = e.motion.x;
                mouseY = e.motion.y;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);

        drawCube(renderer, cube, angleX, angleY);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();

    return 0;
}
