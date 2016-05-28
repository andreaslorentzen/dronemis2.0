//
// Created by hippomormor on 5/28/16.
//

#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "GUI.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"


void GUI::video(sensor_msgs::ImageConstPtr img)
{

      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

      cv::imshow("TEST", cv_ptr->image);
      cv::waitKey(10);

}