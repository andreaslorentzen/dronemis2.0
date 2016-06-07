//
// Created by hippomormor on 6/4/16.
//

#ifndef PROJECT_CV_HANDLER_H
#define PROJECT_CV_HANDLER_H
#include "Cascade.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"

class CV_Handler {

private:
    Cascade *cascade;

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
