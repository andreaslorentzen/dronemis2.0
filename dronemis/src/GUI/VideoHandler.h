//
// Created by hippomormor on 5/28/16.
//

#ifndef PROJECT_VIDEOHANDLER_H
#define PROJECT_VIDEOHANDLER_H


#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class VideoHandler {

private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber video_subscriber;
    std::string video_channel;
    void video(sensor_msgs::ImageConstPtr img);

public:
    VideoHandler(void);
    virtual ~VideoHandler(void);
};


#endif //PROJECT_VIDEOHANDLER_H
