//
// Created by hippomormor on 5/28/16.
//

#ifndef PROJECT_VIDEOHANDLER_H
#define PROJECT_VIDEOHANDLER_H


#include <string>
#include <ros/ros.h>
#include "GUI.h"
#include <cv_bridge/cv_bridge.h>

class VideoHandler {
private:
    GUI *gui;

    ros::NodeHandle nodeHandle;
    ros::Subscriber video_subscriber;
    std::string video_channel;

public:
    VideoHandler(void);
    virtual ~VideoHandler(void);

    void video(const sensor_msgs::ImageConstPtr img);
    void runGUI();
    const ros::Subscriber &getVideo_subscriber() const {
        return video_subscriber;
    }
};


#endif //PROJECT_VIDEOHANDLER_H
