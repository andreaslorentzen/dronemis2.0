//
// Created by thomas on 6/9/16.
//

#ifndef PROJECT_QR_H
#define PROJECT_QR_H


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>

class QR {
private:

public:
    QR(CV_Handler cv);
    int checkCV(void);
};

#endif //PROJECT_QR_H
