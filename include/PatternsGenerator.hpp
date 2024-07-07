#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <filesystem>
#include <string>

template<int dictionaryId>
class PatternsGenerator {
    private:
        int imgSize;
        int markerSize;
        cv::Ptr<cv::aruco::Dictionary> arucoDict;
        std::vector<int> markerIds;

        cv::Mat centerMarkerOnWhiteImage(const cv::Mat& marker) {
            // Crear una imagen blanca de imgSize x imgSize
            cv::Mat white_image = cv::Mat::ones(imgSize, imgSize, CV_8UC3) * 255;

            // Convertir el marcador a color
            cv::Mat markerColor;
            cv::cvtColor(marker, markerColor, cv::COLOR_GRAY2BGR);

            // Coordenadas para centrar el marcador en la imagen blanca
            int top_left_x = (imgSize - markerSize) / 2;
            int top_left_y = (imgSize - markerSize) / 2;

            // Colocar el marcador en la imagen blanca
            markerColor.copyTo(white_image(cv::Rect(top_left_x, top_left_y, markerSize, markerSize)));

            return white_image;
        }

        cv::Mat centerMarkerOnPastelBackground(const cv::Mat& marker, cv::RNG& rng) {
            // Crear una imagen de fondo pastel de imgSize x imgSize
            cv::Mat pastel_background = cv::Mat::zeros(imgSize, imgSize, CV_8UC3);
            
            // Generar colores pastel para el fondo
            cv::Scalar background_color = cv::Scalar(rng.uniform(150, 256), rng.uniform(150, 256), rng.uniform(150, 256));
            pastel_background.setTo(background_color);

            // Convertir el marcador a blanco y negro
            cv::Mat marker_bw;
            cv::cvtColor(marker, marker_bw, cv::COLOR_GRAY2BGR);

            // Coordenadas para centrar el marcador en la imagen de fondo pastel
            int top_left_x = (imgSize - markerSize) / 2;
            int top_left_y = (imgSize - markerSize) / 2;

            // Colocar el marcador en la imagen de fondo pastel
            marker_bw.copyTo(pastel_background(cv::Rect(top_left_x, top_left_y, markerSize, markerSize)));

            return pastel_background;
        }


    public:
        PatternsGenerator(int imgSize, int markerSize, const std::vector<int>& ids){
            this->imgSize = imgSize;
            this->markerSize = markerSize;
            this->markerIds = ids;
            this->arucoDict = new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
        }

        void generateMarkers() {
            // Generador de números aleatorios para colores pastel
            cv::RNG rng(12345);

            for (int markerId : markerIds) {
                // Generar el marcador
                cv::Mat marker;
                cv::aruco::generateImageMarker(*arucoDict, markerId, markerSize, marker);

                // Centrar el marcador en un fondo pastel diferente
                cv::Mat pastel_marker = centerMarkerOnPastelBackground(marker, rng);

                // Crear la carpeta patterns si no existe
                if (!std::filesystem::exists("patterns")) {
                    std::filesystem::create_directory("patterns");
                }

                // Guardar la imagen del marcador
                std::string filename = "patterns/marker" + std::to_string(markerId) + ".png";
                cv::imwrite(filename, pastel_marker);
            }
        }
};
