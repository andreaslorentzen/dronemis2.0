//
// Created by hippomormor on 5/28/16.
//

#include <std_msgs/String.h>
#include "VideoHandler.h"

CV_Handler *cv_handler;

VideoHandler::VideoHandler(CV_Handler* cvHandler){

    cv_handler = cvHandler;

    video_channel = nodeHandle.resolveName("ardrone/image_raw");

    video_subscriber = nodeHandle.subscribe(video_channel,5, &VideoHandler::video, this);
}


VideoHandler::~VideoHandler(void){

}


void VideoHandler::video(sensor_msgs::ImageConstPtr img) {
    // Send video-feed to CV_Handler
    cv_handler->video(img);
}


void VideoHandler::swapCam() {
    cam_service = nodeHandle.serviceClient<std_srvs::Empty>(nodeHandle.resolveName("ardrone/togglecam"),1);
    cam_service.call(toggleCam_srv_srvs);
}
