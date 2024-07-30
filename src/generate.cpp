#include "PatternsGenerator.hpp"

int main() {
    std::vector<int> markerIds = {25, 30, 35};
    PatternsGenerator<cv::aruco::DICT_4X4_50> generator(400, 200, markerIds);
    generator.generateMarkers();

    // Generate markers from 1 to 10
    std::vector<int> markerIds2 = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 40, 41};
    PatternsGenerator<cv::aruco::DICT_4X4_50> generator2(400, 200, markerIds2);
    generator2.generateMarkers();
    
    // Generate marker 11 y 12 together
    std::vector<int> markerIds3 = {11, 12, 13, 14};
    PatternsGenerator<cv::aruco::DICT_4X4_50> generator3(400, 200, markerIds3);
    generator3.generateFourMarkersInRowRectangularImage();
    return 0;
}