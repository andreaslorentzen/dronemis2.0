//
// Created by hippomormor on 5/28/16.
//

#include "VideoHandler.h"
#include "../OpenCv/CV_Handler.h"

CV_Handler *cv_handler;


VideoHandler::VideoHandler(void){

    cv_handler = new CV_Handler();

    video_channel_front = nodeHandle.resolveName("ardrone/front/image_raw");
    video_channel_bottom = nodeHandle.resolveName("ardrone/bottom/image_raw");

    video_subscriber = nodeHandle.subscribe(video_channel_front,10, &VideoHandler::video, this);
}


VideoHandler::~VideoHandler(void){
    delete(cv_handler);
}


void VideoHandler::video(sensor_msgs::ImageConstPtr img) {
    // Send video-feed to CV_Handler
    cv_handler->video(img);

}


void VideoHandler::swapCam(bool frontCam) {
    if (frontCam) {
        video_subscriber.shutdown();
        video_subscriber = nodeHandle.subscribe(video_channel_front,10, &VideoHandler::video, this);
    }
    else {
        video_subscriber.shutdown();
        video_subscriber = nodeHandle.subscribe(video_channel_bottom, 10, &VideoHandler::video, this);
    }
    ROS_INFO("%d", frontCam);
}
