#include "PatternsGenerator.hpp"

int main() {
    std::vector<int> markerIds = {25, 30, 35};
    PatternsGenerator<cv::aruco::DICT_4X4_50> generator(400, 200, markerIds);
    generator.generateMarkers();
    
    return 0;
}