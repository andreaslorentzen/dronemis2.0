//
// Created by hippomormor on 6/4/16.
//

#include "Cascade.h"

using namespace cv;

Cascade::Cascade(void) {

    if(!cascade_classifier.load("../workspaces/dronemis_ws/src/dronemis/src/OpenCv/cascades/cascade700.xml")){
        ROS_INFO("ERROR LOADING DEFAULT CASCADE..");
    };
}

Cascade::~Cascade(void) {

}

bool Cascade::setCascade(const int cascadeNumber) {

    String cascadeName;

    switch(cascadeNumber) {
        case 0:
            cascadeName = "../workspaces/dronemis_ws/src/dronemis/src/OpenCv/cascades/cascade700.xml";
            break;
        case 1:
            cascadeName = "../workspaces/dronemis_ws/src/dronemis/src/OpenCv/cascades/cascade.xml";
            break;
        case 2:
            cascadeName = "../workspaces/dronemis_ws/src/dronemis/src/OpenCv/cascades/cascadeX.xml";
            break;
    }

    if(!cascade_classifier.load(cascadeName)){
        ROS_INFO("ERROR LOADING CASCADE..");
        return false;
    };

    return true;
}


std::vector<Cascade::cubeInfo> Cascade::checkCascade(cv::Mat image) {
    std::vector<Rect> cubes;
    std::vector<cubeInfo> cascades;
    Mat frame_modified;

    equalizeHist(image, frame_modified);

    cascade_classifier.detectMultiScale( frame_modified, cubes, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(20, 20) );

    for( size_t i = 0; i < cubes.size(); i++ )
    {
        cubeInfo cube;
        Point center( cubes[i].x + cubes[i].width*0.5, cubes[i].y + cubes[i].height*0.5 );
        cube.x = center.x;
        cube.y = center.y;
        cube.width = cubes[i].width;
        cube.height = cubes[i].height;
        cascades.push_back(cube);
    }
    std::cout << cubes.size() << std::endl;

    return cascades;
}