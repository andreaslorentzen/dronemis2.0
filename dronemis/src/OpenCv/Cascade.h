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

class Cascade {

private:
    cv::CascadeClassifier cascade_classifier;

public:
    void Cascade::Cascade();
    bool Cascade::setCascade(const cv::String cascadeName);
    int checkCascade(sensor_msgs::ImageConstPtr img);
};

#endif //PROJECT_CASCADE_H
