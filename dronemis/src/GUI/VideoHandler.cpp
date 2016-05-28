//
// Created by hippomormor on 5/28/16.
//

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "VideoHandler.h"
#include "GUI.h"

VideoHandler::VideoHandler(){
    gui = new GUI();
    video_channel = nodeHandle.resolveName("ardrone/image_raw");
    video_subscriber = nodeHandle.subscribe(video_channel,10, &VideoHandler::video, this);

}
VideoHandler::~VideoHandler()
{
    delete gui;
}
void VideoHandler::video(const sensor_msgs::ImageConstPtr img)
{
    gui->video(img);

}