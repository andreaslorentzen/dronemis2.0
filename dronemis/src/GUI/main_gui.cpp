//
// Created by hippomormor on 5/28/16.
//
//
#include <ros/init.h>
#include "VideoHandler.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dronemis_gui");

    ROS_INFO("Starting Dronemis!!!! Be ready!");

    VideoHandler *videoNode = new VideoHandler();
    videoNode->runGUI();

    ros::spin();

    return 0;
}