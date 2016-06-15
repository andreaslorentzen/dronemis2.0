//
// Created by hippomormor on 6/4/16.
//

#ifndef PROJECT_CV_HANDLER_H
#define PROJECT_CV_HANDLER_H
#include "Cascade.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "boost/thread.hpp"
#include "cvd/thread.h"
#include "cvd/image.h"
#include "cvd/byte.h"
#include <cvd/image_io.h>
#include <cvd/rgb.h>
#include <std_srvs/Empty.h>


class CV_Handler {


private:
    void show(void);
    ros::NodeHandle nodeHandle;
    ros::ServiceClient cam_service;
    ros::NodeHandle cam_channel;
    std_srvs::Empty toggleCam_srv_srvs;

public:
    bool cascade_image_ready;
    bool greySelected;
    bool frontCamSelected;
    boost::condition_variable  new_frame_signal;
    boost::mutex new_frame_signal_mutex;
    boost::condition_variable  new_cascade_signal;
    boost::mutex new_cascade_signal_mutex;
    CVD::Image<CVD::byte> storedImageBW;
    CVD::Image<CVD::byte> workImageBW;
    CVD::Image<CVD::Rgb <float> > storedImage;
    CVD::Image<CVD::Rgb <float> > workImage;

    CV_Handler(void);
    void run(void);
    virtual ~CV_Handler(void);

    void video(sensor_msgs::ImageConstPtr img);
    std::vector<Cascade::cubeInfo> checkCubes(void);
    void swapCam(bool frontCam);
};



#endif //PROJECT_CV_HANDLER_H
