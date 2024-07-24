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
            this->arucoDict = new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(dictionaryId));
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

        void generateFourMarkersInOneImage() {
            // Generar una imagen blanca para contener los cuatro marcadores
            cv::Mat combined_image = cv::Mat::ones(imgSize * 2, imgSize * 2, CV_8UC3) * 255;

            // Generador de números aleatorios para colores pastel
            cv::RNG rng(12345);

            // Iterar sobre los primeros cuatro marcadores en markerIds
            for (size_t i = 0; i < std::min<size_t>(4, markerIds.size()); ++i) {
                int markerId = markerIds[i];

                // Generar el marcador
                cv::Mat marker;
                cv::aruco::generateImageMarker(*arucoDict, markerId, markerSize, marker);

                // Centrar el marcador en un fondo pastel diferente
                cv::Mat pastel_marker = centerMarkerOnPastelBackground(marker, rng);

                // Calcular las coordenadas para colocar el marcador en la imagen combinada
                int row = i / 2;
                int col = i % 2;
                cv::Mat roi = combined_image(cv::Rect(col * imgSize, row * imgSize, imgSize, imgSize));

                // Copiar el marcador pastel en la región de interés (ROI) correspondiente
                pastel_marker.copyTo(roi);
            }

            // Crear la carpeta patterns si no existe
            if (!std::filesystem::exists("patterns")) {
                std::filesystem::create_directory("patterns");
            }

            // Guardar la imagen combinada con los cuatro marcadores
            std::string filename = "patterns/four_markers.png";
            cv::imwrite(filename, combined_image);
        }

        void generateFourMarkersInRowRectangularImage(){
            // Generar una imagen blanca para contener los cuatro marcadores
            cv::Mat combined_image = cv::Mat::ones(imgSize, imgSize * 4, CV_8UC3) * 255;

            // Generador de números aleatorios para colores pastel
            cv::RNG rng(12345);

            // Iterar sobre los primeros cuatro marcadores en markerIds
            for (size_t i = 0; i < std::min<size_t>(4, markerIds.size()); ++i) {
                int markerId = markerIds[i];

                // Generar el marcador
                cv::Mat marker;
                cv::aruco::generateImageMarker(*arucoDict, markerId, markerSize, marker);

                // Centrar el marcador en un fondo pastel diferente
                cv::Mat pastel_marker = centerMarkerOnPastelBackground(marker, rng);

                // Calcular las coordenadas para colocar el marcador en la imagen combinada
                cv::Mat roi = combined_image(cv::Rect(i * imgSize, 0, imgSize, imgSize));

                // Copiar el marcador pastel en la región de interés (ROI) correspondiente
                pastel_marker.copyTo(roi);
            }

            // Crear la carpeta patterns si no existe
            if (!std::filesystem::exists("patterns")) {
                std::filesystem::create_directory("patterns");
            }

            // Guardar la imagen combinada con los cuatro marcadores
            std::string filename = "patterns/four_markers_row.png";
            cv::imwrite(filename, combined_image);
        }
};
