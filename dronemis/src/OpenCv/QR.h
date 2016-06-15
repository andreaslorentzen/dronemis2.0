//
// Created by thomas on 6/9/16.
//

#ifndef PROJECT_QR_H
#define PROJECT_QR_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include "CV_Handler.h"

class QR {
private:
    CV_Handler *cvHandler;
    struct QRCodes {
        int x;
        int y;
        cv::String name;
    };
    struct QRCodes QRWallCode[25];
    float distanceToQR[200];
    double calculateDistance(int pixel);
    void initializeQR(void);

public:
    QR(CV_Handler *cv);
    int checkQR(void);
};

#endif //PROJECT_QR_H
