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

public:
    int kernelState = 2;
    cv::Mat kernel;

    struct {
        int state;
        int kernel;
    }data;

    struct {
        int iLowH = 150;
        int iHighH = 255;

        int iLowS = 119;
        int iHighS = 255;

        int iLowV = 0;
        int iHighV = 255;
    }redFilter;

    struct {
        int iLowH = 0;
        int iHighH = 118;

        int iLowS = 134;
        int iHighS = 255;

        int iLowV = 0;
        int iHighV = 129;
    }greenFilterDark;

    struct {
        int iLowH = 46;
        int iHighH = 95;

        int iLowS = 56;
        int iHighS = 255;

        int iLowV = 0;
        int iHighV = 165;
    }greenFilter;

    Color(void);
    virtual ~Color(void);
    cv::Mat checkColorsRed(std::vector<Cascade::cubeInfo> *cubes, cv::Mat image);
    cv::Mat checkColorsGreen(std::vector<Cascade::cubeInfo> *cubes, cv::Mat image);
    std::vector<Cascade::cubeInfo> checkColors(std::vector<Cascade::cubeInfo> *cubes, cv::Mat image);
    cv::Mat TESTcheckColorsGreen(cv::Mat image);
    cv::Mat TESTcheckColorsRed(cv::Mat image);
};


#endif //PROJECT_COLOR_H
