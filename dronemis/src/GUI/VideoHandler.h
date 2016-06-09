//
// Created by hippomormor on 5/28/16.
//

#ifndef PROJECT_VIDEOHANDLER_H
#define PROJECT_VIDEOHANDLER_H

#include "std_msgs/Empty.h"
#include "stdlib.h"
#include "std_srvs/Empty.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "../OpenCv/CV_Handler.h"

class VideoHandler {

private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber video_subscriber;
    ros::ServiceClient cam_service;
    std::string video_channel;
    ros::NodeHandle cam_channel;
    std_srvs::Empty toggleCam_srv_srvs;
    void video(sensor_msgs::ImageConstPtr img);

public:
    VideoHandler(CV_Handler * cvHandler);
    virtual ~VideoHandler(void);
    void swapCam();
};


#endif //PROJECT_VIDEOHANDLER_H
