//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"



CV_Handler::CV_Handler() {
    cascade = new Cascade();
    video = new VideoHandler();
}

CV_Handler::~CV_Handler() {
   delete(cascade);
   delete(video);
}


CV_Handler::cascadeInfo** CV_Handler::checkColors(bool camera) {
    return NULL;
}


CV_Handler::cascadeInfo** CV_Handler::checkCascades(bool camera) {
    return NULL;
}

