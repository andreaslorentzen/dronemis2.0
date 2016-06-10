//
// Created by hippomormor on 6/4/16.
//

#include "Cascade.h"

#include "../flightControl/blindFlight.h"

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


cv::Mat Cascade::checkCascade(cv::Mat image) {
    std::vector<Rect> cubes;
    Mat frame_modified;

    //cvtColor( image, frame_modified, CV_BGR2GRAY );
    equalizeHist( frame_modified, frame_modified );

    //-- Detect cubes
    cascade_classifier.detectMultiScale( frame_modified, cubes, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(20, 20) );

    for( size_t i = 0; i < cubes.size(); i++ )
    {
        Point center( cubes[i].x + cubes[i].width*0.5, cubes[i].y + cubes[i].height*0.5 );
        ellipse( frame_modified, center, Size( cubes[i].width*0.5, cubes[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
    cout << cubes.size() << endl;

    return frame_modified;
}