//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"
#include "../GUI/VideoHandler.h"

VideoHandler *videohandler;

CV_Handler::CV_Handler(void)  {

}

void CV_Handler::run(void){
    cascade = new Cascade();
    videohandler = new VideoHandler();
}

CV_Handler::~CV_Handler(void) {
    delete(cascade);
    delete(videohandler);
}

void CV_Handler::video(sensor_msgs::ImageConstPtr img) {

    // Convert from ROS image message to OpenCV Mat with cv_bridge
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

    cv::imshow("TEST", cv_ptr->image);
    cv::waitKey(10);
}

CV_Handler::cascadeInfo** CV_Handler::checkColors(bool camera) {
    return NULL;
}


CV_Handler::cascadeInfo** CV_Handler::checkCascades(bool camera) {
    return NULL;
}

