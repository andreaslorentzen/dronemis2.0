//
// Created by hippomormor on 6/4/16.
//

#ifndef PROJECT_CV_HANDLER_H
#define PROJECT_CV_HANDLER_H

#include "Cascade.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "cvd/thread.h"
#include "cvd/image.h"
#include "cvd/byte.h"
#include "../navdata/Nav.h"
#include <cvd/image_io.h>
#include <cvd/rgb.h>
#include <std_srvs/Empty.h>
#include <mutex>
#include <thread>
#include "../debug.h"

class CV_Handler {

private:
    struct plottedCube{
        double x;
        double y;
    };

    int thresh = 64;
    int missingBoxFrames = 0;
    ros::NodeHandle nodeHandle;
    ros::ServiceClient cam_service;
    ros::Subscriber video_subscriber;
    std_srvs::Empty toggleCam_srv_srvs;
    std::string video_channel;
    std::vector<int> boxVector;
    std::vector<plottedCube> plottedCubes;
    cv::String map_name = "../workspaces/dronemis_ws/src/dronemis/src/OpenCv/map.png";
    cv::String output_map_name = "../workspaces/dronemis_ws/src/dronemis/src/OpenCv/output_map.jpg";
    cv::Mat map;
    double findMedian(std::vector<int> vec);
    void video(sensor_msgs::ImageConstPtr img);
    void show(void);
    void paintCube(cv::Point center, std::string type);
    void calculatePosition(std::vector<Cascade::cubeInfo> *cubes);

public:
    bool imageReady;
    std::mutex cascadeMutex;
    CVD::Image<CVD::byte> storedImageBW;
    CVD::Image<CVD::Rgb <float> > storedImage;

    CV_Handler(void);
    virtual ~CV_Handler(void);
    void run(Nav *nav);
    void swapCam(bool frontCam);
    void checkCubes(void);
    cv::Mat checkBox(void);
};

#endif //PROJECT_CV_HANDLER_H
