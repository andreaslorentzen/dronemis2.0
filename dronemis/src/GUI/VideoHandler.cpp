//
// Created by hippomormor on 5/28/16.
//

#include "VideoHandler.h"

VideoHandler::VideoHandler(void){
    video_channel = nodeHandle.resolveName("ardrone/image_raw");
    video_subscriber = nodeHandle.subscribe(video_channel,10, &VideoHandler::video, this);
}

VideoHandler::~VideoHandler(void){
    delete gui;
}

void VideoHandler::runGUI(void){
    gui = new GUI();
}

void VideoHandler::video(const sensor_msgs::ImageConstPtr img) {
    gui->video(img);
}

