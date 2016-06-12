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

    struct {
        int iLowH = 173;
        int iHighH = 196;

        int iLowS = 83;
        int iHighS = 140;

        int iLowV = 44;
        int iHighV = 93;
    }redFilter;

    struct {
        int iLowH = 32;
        int iHighH = 50;

        int iLowS = 21;
        int iHighS = 63;

        int iLowV = 31;
        int iHighV = 67;
    }greenFilter;

    struct {
        int iLowH = 0;
        int iHighH = 255;

        int iLowS = 0;
        int iHighS = 255;

        int iLowV = 0;
        int iHighV = 255;
    }allFilter;



public:
    Color(void);
    virtual ~Color(void);

    std::vector<Cascade::cubeInfo> checkColors(std::vector<Cascade::cubeInfo> cubes, cv::Mat image);
    cv::Mat checkColorsTest(std::vector<Cascade::cubeInfo> cubes, cv::Mat image);
};


#endif //PROJECT_COLOR_H
