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

class CV_Handler {


private:
    CVD::Image<CVD::Rgb <float> > storedImage;
    CVD::Image<CVD::Rgb <float> > workImage;
    int mimFrameTime;
    int mimFrameTime_workingCopy;
    unsigned int mimFrameSEQ;
    unsigned int mimFrameSEQ_workingCopy;
    ros::Time mimFrameTimeRos;
    ros::Time mimFrameTimeRos_workingCopy;
    int frameWidth, frameHeight;

    Cascade *cascade;

    boost::condition_variable  new_frame_signal;
    boost::mutex new_frame_signal_mutex;

public:

    struct cascadeInfo {
       int x;
       int y;
       int z;
       unsigned char color;
    };

    CV_Handler(void);
    void run(void);
    virtual ~CV_Handler(void);
    void video(sensor_msgs::ImageConstPtr img);
    cascadeInfo** checkColors(bool camera);
    cascadeInfo** checkCascades(bool camera);
};



#endif //PROJECT_CV_HANDLER_H
