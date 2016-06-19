//
// Created by hippomormor on 6/4/16.
//

#ifndef PROJECT_CASCADE_H
#define PROJECT_CASCADE_H

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <iostream>

class Cascade {

private:
    cv::CascadeClassifier cascade_classifier;

public:
    struct cubeInfo{
        int x;
        int y;
        double xDist;
        double yDist;
        double width;
        double height;
        std::string color;
        cv::Mat image;
    };

    Cascade(void);
    virtual ~Cascade(void);
    bool setCascade(const int cascadeNumber);

    std::vector<Cascade::cubeInfo> checkCascade(cv::Mat image);
};

#endif //PROJECT_CASCADE_H
