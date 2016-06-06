//
// Created by hippomormor on 6/4/16.
//

#ifndef PROJECT_CV_HANDLER_H
#define PROJECT_CV_HANDLER_H

#include "Cascade.h"
#include "../GUI/VideoHandler.h"

class CV_Handler {

private:
    Cascade *cascade;
    VideoHandler *video;

public:
    struct cascadeInfo {
       int x;
       int y;
       int z;
       unsigned char color;
    };

    CV_Handler(void);
    virtual ~CV_Handler();
    cascadeInfo** checkColors(bool camera);
    cascadeInfo** checkCascades(bool camera);
};


#endif //PROJECT_CV_HANDLER_H
