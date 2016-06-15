//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"
#include "../GUI/VideoHandler.h"
#include "ros/callback_queue.h"
#include "Color.h"
#include "QR.h"

#define CASCADE_FRAMES 10

VideoHandler *videohandler;
Cascade *cascade;
Color *color;

CV_Handler::CV_Handler(void) {

}

void CV_Handler::run(void) {
    frontCamSelected = false;
    greySelected = false;
    cascade = new Cascade();
    color = new Color();
    videohandler = new VideoHandler(this);
}


CV_Handler::~CV_Handler(void) {
    delete(cascade);
    delete(videohandler);
    delete(color);
}


void CV_Handler::video(sensor_msgs::ImageConstPtr img) {

    size_t size = img->width * img->height;

    // Convert from ROS image message to OpenCV Mat with cv_bridge (unsigned 8 bit MONO)
    cv_bridge::CvImagePtr cv_ptrBw = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

    // Save size constraints to data structure
    storedImageBW.resize(CVD::ImageRef(img->width, img->height));

    // Convert from ROS image message to OpenCV Mat with cv_bridge (unsigned 8 bit BGR)
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    storedImage.resize(CVD::ImageRef(img->width, img->height));

    boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

    // Copy image to CVD data structure
    memcpy(storedImageBW.data(), cv_ptrBw->image.data, size);
    memcpy(storedImage.data(), cv_ptr->image.data, size * 3);

    cascade_image_ready = true;

     //Unlock mutex
    lock.unlock();
    new_frame_signal.notify_all();


    //show();
}


void CV_Handler::show(void) {

    cv::Mat image;

    if (greySelected) {
        // Convert CVD byte array to OpenCV matrix (use CV_8UC1 format - unsigned 8 bit MONO)
        cv::Mat imageBW(storedImageBW.size().y,
                        storedImageBW.size().x,
                        CV_8UC1,
                        storedImageBW.data());
        image = imageBW;
    } else {
        // Convert CVD byte array to OpenCV matrix (use CV_8UC3 format - unsigned 8 bit BGR 3 channel)
        cv::Mat imageBGR(storedImage.size().y,
                         storedImage.size().x,
                         CV_8UC3,
                 storedImage.data());
        image = imageBGR;
        //image = color->checkColorsTest(std::vector<Cascade::cubeInfo>(), image);
    }

    cv::imshow("VideoMis", image);
    if (cv::waitKey(10) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        system("kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')");
}


void CV_Handler::swapCam(bool frontCam) {
    if (frontCamSelected != frontCam)
        videohandler->swapCam();
    frontCamSelected = frontCam;
}


std::vector<Cascade::cubeInfo> CV_Handler::checkCubes(void) {
    int frameCount = 0;

    typedef std::vector<Cascade::cubeInfo> cascadeArray;
    std::vector<cascadeArray> cascades;

    ros::Rate r(10); // 10 hz

    cv::Mat processedImage;

    boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);
    lock.unlock();

    while (ros::ok()) {

        if (cascade_image_ready) {
            lock.lock();

            cv::Mat imageBW(storedImageBW.size().y,
                            storedImageBW.size().x,
                            CV_8UC1,
                            storedImageBW.data());

            cv::Mat image(storedImage.size().y,
                          storedImage.size().x,
                          CV_8UC3,
                          storedImage.data());
                          
            cascade_image_ready = false;

            lock.unlock();
            new_frame_signal.notify_all();

            cascades.push_back(color->checkColors(cascade->checkCascade(imageBW), image));

            if (++frameCount == CASCADE_FRAMES)
                break;

            r.sleep();
        } else
            new_frame_signal.wait(lock);
    }

    int biggestArray = 0;

    for (unsigned int i = 0; i < cascades.size(); i++) {
       if (cascades[i].size() > cascades[biggestArray].size())
           biggestArray = i;
    }


    std::cout << "The biggest array is nr. " << biggestArray << std::endl;







/*





    storedImage.resize(CVD::ImageRef(processedImage.cols, processedImage.rows));

    size_t size = processedImage.cols * processedImage.rows;

    lock.lock();

    memcpy(storedImage.data(), processedImage.data,  size*3);
    //greySelected = false;

    lock.unlock();
    new_frame_signal.notify_all();
*/
    return std::vector<Cascade::cubeInfo>();
}
