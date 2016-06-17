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

class CV_Handler {


private:
    int thresh = 196;
    ros::NodeHandle nodeHandle;
    ros::ServiceClient cam_service;
    ros::Subscriber video_subscriber;
    std_srvs::Empty toggleCam_srv_srvs;
    std::string video_channel;

    void video(sensor_msgs::ImageConstPtr img);
    void show(void);
    void swapCam(bool frontCam);
    std::vector<Cascade::cubeInfo> calculatePosition(std::vector<Cascade::cubeInfo> cubes);

public:
    struct boxCordsStruct {
        int x;
        int y;
    }boxCords;

    bool graySelected;
    bool imageReady;
    bool frontCamSelected;
    std::mutex cascadeMutex;
    CVD::Image<CVD::byte> storedImageBW;
    CVD::Image<CVD::Rgb <float> > storedImage;
    CVD::Image<CVD::byte> workImageBW;
    CVD::Image<CVD::Rgb <float> > workImage;

    CV_Handler(void);
    virtual ~CV_Handler(void);

    void run(Nav *nav);
    std::vector<Cascade::cubeInfo> checkCubes(void);
    boxCordsStruct checkBox(void);
};

#endif //PROJECT_CV_HANDLER_H
