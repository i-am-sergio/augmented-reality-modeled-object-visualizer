#pragma once

#include <opencv2/opencv.hpp>

class LoadCamera {
    private: 
        cv::VideoCapture cap;
        cv::Mat frame;
    
    public:
        LoadCamera(){}
        ~LoadCamera(){
            if (cap.isOpened()){
                cap.release();
            }
        }
        
        bool openCamera(int cameraIndex = 0){
            cap.open(cameraIndex);
            return cap.isOpened();
        }

        void showCamera(){
            if (!cap.isOpened()){
                std::cout << "Camera is not opened" << std::endl;
                return;
            }

            while (true){
                this->cap >> frame;
                if (frame.empty()) break;

                cv::imshow("Camera", frame);

                // stop to press 'q' or 'esc'
                if (cv::waitKey(10) == 27 || cv::waitKey(10) == 'q'){
                    break;
                }
                // stop to clicked x button
                if (cv::getWindowProperty("Camera", cv::WND_PROP_AUTOSIZE) == -1){
                    break;
                }
            }

        }
        cv::Mat getFrame();
};