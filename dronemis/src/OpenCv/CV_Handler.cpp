//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"


using namespace cv;


CV_Handler::CV_Handler() {
    cascade = new Cascade();

}

CV_Handler::~CV_Handler() {
    delete(cascade);
}

