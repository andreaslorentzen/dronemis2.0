//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"



CV_Handler::CV_Handler(const sensor_msgs::ImageConstPtr img) {
    cascade = new Cascade();

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

    cv::imshow("TEST", cv_ptr->image);
    cv::waitKey(10);
}

CV_Handler::~CV_Handler(void) {
   delete(cascade);
}


CV_Handler::cascadeInfo** CV_Handler::checkColors(bool camera) {
    return NULL;
}


CV_Handler::cascadeInfo** CV_Handler::checkCascades(bool camera) {
    return NULL;
}

