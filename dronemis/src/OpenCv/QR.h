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
#include "../debug.h"

struct DronePos {
    int x;
    int y;
    int relativeX;
    int relativeY;
    int heading;
    bool positionLocked;
    int numberOfQRs;
    double cameraPointing;
    double angle;
    void resetCoordinates(){
        x, y, relativeX, relativeY, numberOfQRs, positionLocked, heading, cameraPointing, angle = 0;
    }
};

class QR {
public:
    QR(CV_Handler *cv);
    DronePos checkQR(void);

private:
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

    struct QRCodes QRWallCode[25];
    struct DronePos DronePosition;
    struct DronePos RoomDronePosition;
    float distanceToQR[200];

    void initializeQR(void);
    void calculateRoomDronePostition(std::string QRname, int relativeX, int relativeY, bool positionLocked,
                                     int numberOfQRs, double cameraPointing, double angle);
    double calculateDistanceToQR(int pixel);
};

#endif //PROJECT_QR_H
