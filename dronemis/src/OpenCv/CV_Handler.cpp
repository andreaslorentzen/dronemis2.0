//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"
#include "../GUI/VideoHandler.h"


VideoHandler *videohandler;


CV_Handler::CV_Handler(void) {

}


void CV_Handler::run(void) {
    cascade = new Cascade();
    videohandler = new VideoHandler();
}


CV_Handler::~CV_Handler(void) {
    delete (cascade);
    delete (videohandler);
}


void CV_Handler::video(sensor_msgs::ImageConstPtr img) {


    // Convert from ROS image message to OpenCV Mat with cv_bridge (unsigned 8 bit BGR)
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

    // Lock with unique lock (can be closed/opened remotely)
    boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

    // Save size constraints to data structure
    storedImage.resize(CVD::ImageRef(img->width, img->height));

    // Copy image to CVD data structure
    size_t size = img->width * img->height;
    memcpy(storedImage.data(), cv_ptr->image.data,  size * 3);

    // Convert CVD byte array to OpenCV matrix (use CV_8UC3 format - unsigned 8 bit bgr 3 channel)
    cv::Mat image(img->height, img->width,CV_8UC3, storedImage.data());

    // Unlock mutex
    lock.unlock();
    new_frame_signal.notify_all();

    cv::imshow("TEST", image);
    cv::waitKey(10);

}


CV_Handler::cascadeInfo **CV_Handler::checkColors(bool camera) {
    return NULL;
}


CV_Handler::cascadeInfo **CV_Handler::checkCascades(bool camera) {
    //boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);
    return NULL;
}

