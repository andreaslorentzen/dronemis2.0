//
// Created by hippomormor on 5/28/16.
//
#include "GUI.h"

GUI::GUI(sensor_msgs::ImageConstPtr img){

      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB16);

      cv::imshow("TEST", cv_ptr->image);
      cv::waitKey(10);
}