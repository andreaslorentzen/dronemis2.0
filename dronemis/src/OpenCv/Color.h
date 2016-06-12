//
// Created by hippomormor on 6/4/16.
//

#ifndef PROJECT_COLOR_H
#define PROJECT_COLOR_H

#include "Cascade.h"
#include <iostream>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class Color {

private:
    struct redFilter {
        int iLowH = 102;
        int iHighH = 206;

        int iLowS = 129;
        int iHighS = 225;

        int iLowV = 108;
        int iHighV = 255;
    };

    struct greenFilter {
        int iLowHg = 25;
        int iHighHg = 56;

        int iLowSg = 26;
        int iHighSg = 181;

        int iLowVg = 21;
        int iHighVg = 160;
    };

public:
    Color();
    virtual ~Color();
    cv::Mat  checkColors(std::vector<Cascade::cubeInfo> cubes, cv::Mat image);
};


#endif //PROJECT_COLOR_H
