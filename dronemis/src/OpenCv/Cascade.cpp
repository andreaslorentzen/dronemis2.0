//
// Created by hippomormor on 6/4/16.
//

#include "Cascade.h"

using namespace cv;


Cascade::Cascade() {

    if(!cascade_classifier.load("cascades/cascade700.xml")){
        ROS_INFO("ERROR LOADING DEFAULT CASCADE..");
    };
}

Cascade::~Cascade() {

    delete(cascade_classifier);
}

bool Cascade::setCascade(const int cascadeNumber) {

    String cascadeName;

    switch(cascadeNumber) {
        case 0:
            cascadeName = "cascades/cascade700.xml";
            break;
        case 1:
            cascadeName = "cascades/cascade.xml";
            break;
        case 2:
            cascadeName = "cascades/cascadeX.xml";
            break;
    }

    if(!cascade_classifier.load(cascadeName)){
        ROS_INFO("ERROR LOADING CASCADE..");
        return false;
    };
}


int Cascade::checkCascade( sensor_msgs::ImageConstPtr img ) {

    std::vector<Rect> cubes;
    Mat frame_modified;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

    //cvtColor( cv_ptr->image, frame_gray, CV_BGR2GRAY ); // hmmm
    equalizeHist( cv_ptr->image, frame_modified );

    //-- Detect cubes
    cascade_classifier.detectMultiScale( frame_modified, cubes, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(20, 20) );

    for( size_t i = 0; i < cubes.size(); i++ )
    {
        Point center( cubes[i].x + cubes[i].width*0.5, cubes[i].y + cubes[i].height*0.5 );
        ellipse( cv_ptr->image, center, Size( cubes[i].width*0.5, cubes[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

    }

    return 0;
}