//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"
#include "../GUI/VideoHandler.h"
#include "ros/callback_queue.h"

VideoHandler *videohandler;
Cascade *cascade;

struct thread_data{
    CV_Handler *cvHandler;
} threadData;

void *show(void *thread_arg);

CV_Handler::CV_Handler(void) {

}

void CV_Handler::run(void) {
    greySelected = false;
    cascade = new Cascade();
    videohandler = new VideoHandler(this);
}


CV_Handler::~CV_Handler(void) {
    delete (cascade);
    delete (videohandler);
}


void CV_Handler::video(sensor_msgs::ImageConstPtr img) {
    cv_bridge::CvImagePtr cv_ptr;
    size_t size = img->width * img->height;

    if (greySelected) {
        // Convert from ROS image message to OpenCV Mat with cv_bridge (unsigned 8 bit MONO)
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

        // Save size constraints to data structure
        storedImageBW.resize(CVD::ImageRef(img->width, img->height));
    } else {
        // Convert from ROS image message to OpenCV Mat with cv_bridge (unsigned 8 bit BGR)
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        storedImage.resize(CVD::ImageRef(img->width, img->height));
    }

    boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

    // Copy image to CVD data structure
    if (greySelected)
        memcpy(storedImageBW.data(), cv_ptr->image.data, size);
    else
        memcpy(storedImage.data(), cv_ptr->image.data, size * 3);

    // Unlock mutex
    lock.unlock();
    new_frame_signal.notify_all();

    pthread_t thread;
    threadData.cvHandler = this;
    pthread_create(&thread, NULL, show, &threadData);

    // spinning this way instead of ros::spin.'
    while(ros::ok())
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

    pthread_exit(NULL);

}


void *show(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;
    cv::Mat image;

    if (thread_data->cvHandler->greySelected) {
        // Convert CVD byte array to OpenCV matrix (use CV_8UC1 format - unsigned 8 bit MONO)
        cv::Mat imageBW(thread_data->cvHandler->storedImageBW.size().y,
                        thread_data->cvHandler->storedImageBW.size().x,
                        CV_8UC1,
                        thread_data->cvHandler->storedImageBW.data());
        image = imageBW;
    } else {
        // Convert CVD byte array to OpenCV matrix (use CV_8UC3 format - unsigned 8 bit BGR 3 channel)
        cv::Mat imageBGR(thread_data->cvHandler->storedImage.size().y,
                         thread_data->cvHandler->storedImage.size().x,
                         CV_8UC3,
                 thread_data->cvHandler->storedImage.data());
        image = imageBGR;
    }

    cv::imshow("VideoMis", image);
    cv::waitKey(10);

    pthread_exit(NULL);
}


void *cascadeHandler(void *thread_arg) {

    pthread_exit(NULL);
}


void CV_Handler::swapCam() {
    videohandler->swapCam();
}


CV_Handler::cascadeInfo **CV_Handler::checkColors(void) {
    return NULL;
}


CV_Handler::cascadeInfo **CV_Handler::checkCascades(void) {

    greySelected = true;

    ros::Rate r(10); // 10 hz

    cv::Mat processedImage;

    int i = 0;
    boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);
    lock.unlock();

    while (ros::ok()) {

        lock.lock();

        cv::Mat imageBW(storedImageBW.size().y,
                    storedImageBW.size().x,
                    CV_8UC1,
                    storedImageBW.data());

        lock.unlock();
        new_frame_signal.notify_all();

        foundCascade = cascade->checkCascade(imageBW);
        if (i++ == 4)
            break;
        r.sleep();
    }

    storedImageBW.resize(CVD::ImageRef(processedImage.cols, processedImage.rows));

    size_t size = processedImage.cols * processedImage.rows;

    lock.lock();

    memcpy(storedImageBW.data(), processedImage.data,  size);

    lock.unlock();
    new_frame_signal.notify_all();

    return NULL;
}
