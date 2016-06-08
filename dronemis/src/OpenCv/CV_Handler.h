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


public:
    boost::condition_variable  new_frame_signal;
    boost::mutex new_frame_signal_mutex;

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
    cascadeInfo** checkColors(void);
    cascadeInfo** checkCascades(void);
    void swapCam(void);
};



#endif //PROJECT_CV_HANDLER_H
