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
    int averageCount = 0;
    int direction;
    double yRatioTemp;
    double yRatioAverage;
    double y1Diversion;
    double y2Diversion;
    double yDiversionAngle;



    CV_Handler *cvHandler;
    struct QRCodes {
        int x;
        int y;
        cv::String name;
    };

    struct DronePos {
        int x;
        int y;
        int heading;
    };


    struct QRCodes QRWallCode[25];
    struct DronePos DronePosition;
    struct DronePos FinalDronePosition;
    float distanceToQR[200];

    void initializeQR(void);
    void calculateFinalDronePostition(std::string QRname);
    double calculateDistanceToQR(int pixel);

public:
    QR(CV_Handler *cv);
    int checkQR(void);
};

#endif //PROJECT_QR_H
