//
// Created by hippomormor on 5/28/16.
//

#ifndef PROJECT_GUI_H
#define PROJECT_GUI_H

#include <ros/ros.h>
#include "GUI.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/Image.h>

class GUI {
public:
    void video(sensor_msgs::ImageConstPtr img);

};


#endif //PROJECT_GUI_H
