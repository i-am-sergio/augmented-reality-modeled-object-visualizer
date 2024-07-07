#include "PatternsGenerator.hpp"

int main() {
    std::vector<int> markerIds = {25, 30, 35};
    PatternsGenerator<cv::aruco::DICT_4X4_50> generator(400, 200, markerIds);
    generator.generateMarkers();

    // Generate markers from 1 to 10
    std::vector<int> markerIds2 = {11, 12, 13, 14, 15, 40, 41};
    PatternsGenerator<cv::aruco::DICT_4X4_50> generator2(400, 200, markerIds2);
    generator2.generateMarkers();
    
    return 0;
}