//
// Created by hippomormor on 5/28/16.
//

#include <ros/init.h>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/highgui.hpp>
#include "VideoHandler.h"

int main(int argc, char **argv)
{
    cv::String face_cascade_name = "/home/hippomormor/Desktop/p.jpg";
    cv::Mat cvIMG = cv::imread(face_cascade_name,CV_LOAD_IMAGE_COLOR);
    cv::imshow("TEST", cvIMG);

    ros::init(argc, argv, "dronemis_gui");

    ROS_INFO("Starting Dronemis!!!! Be ready!");
    VideoHandler videoNode;

    ros::spin();

    return 0;
}
