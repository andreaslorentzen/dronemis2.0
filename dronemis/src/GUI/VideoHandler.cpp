//
// Created by hippomormor on 5/28/16.
//

#include <std_msgs/String.h>
#include "VideoHandler.h"

CV_Handler *cv_handler;

VideoHandler::VideoHandler(CV_Handler* cvHandler){

    cv_handler = cvHandler;

    video_channel = nodeHandle.resolveName("ardrone/image_raw");

    video_subscriber = nodeHandle.subscribe(video_channel,10, &VideoHandler::video, this);

    ros::Rate r(25);
    while(nodeHandle.ok()) {
        ros::spinOnce();
        r.sleep();


    }
}


VideoHandler::~VideoHandler(void){

}


void VideoHandler::video(sensor_msgs::ImageConstPtr img) {
    // Send video-feed to CV_Handler
    cv_handler->video(img);
}

