//
// Created by hippomormor on 5/28/16.
//

#ifndef PROJECT_VIDEOHANDLER_H
#define PROJECT_VIDEOHANDLER_H


#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <ros/ros.h>

class VideoHandler {

private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber video_subscriber;
    std::string video_channel_front;
    std::string video_channel_bottom;
    void video(sensor_msgs::ImageConstPtr img);

public:
    VideoHandler(void);
    virtual ~VideoHandler(void);
    void swapCam(bool frontCam);
};


#endif //PROJECT_VIDEOHANDLER_H
