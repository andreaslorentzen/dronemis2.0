//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"
#include "Color.h"

#define CASCADE_FRAMES 10

Cascade *cascade;
Color *color;

using namespace std;

CV_Handler::CV_Handler(void) {

}


void CV_Handler::run(void) {
    frontCamSelected = true;
    greySelected = false;
    cascade = new Cascade();
    color = new Color();
    video_channel = nodeHandle.resolveName("ardrone/image_raw");
    video_subscriber = nodeHandle.subscribe(video_channel,10, &CV_Handler::video, this);

    ros::Rate r(25);
    while(nodeHandle.ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

CV_Handler::~CV_Handler(void) {
    delete(cascade);
    delete(color);
}


void CV_Handler::video(sensor_msgs::ImageConstPtr img) {
    size_t size = img->width * img->height;

    // Convert from ROS image message to OpenCV Mat with cv_bridge
    cv_bridge::CvImagePtr cv_ptrBw = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

    // Save size constraints to data structure
    storedImageBW.resize(CVD::ImageRef(img->width, img->height));
    storedImage.resize(CVD::ImageRef(img->width, img->height));

    cascadeMutex.lock();

    // Copy image to CVD data structure
    memcpy(storedImageBW.data(), cv_ptrBw->image.data, size);
    memcpy(storedImage.data(), cv_ptr->image.data, size * 3);

    cascadeMutex.unlock();

    show();
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
    cam_service = nodeHandle.serviceClient<std_srvs::Empty>(nodeHandle.resolveName("ardrone/togglecam"),1);
    if (frontCamSelected != frontCam) {
        cam_service.call(toggleCam_srv_srvs);
        frontCamSelected = frontCam;
    }
}


std::vector<Cascade::cubeInfo> CV_Handler::checkCubes(void) {
    int frameCount = 0;
    int biggestArray = 0;
    typedef std::vector<Cascade::cubeInfo> cascadeArray;
    std::vector<cascadeArray> cascades(1);

    while (true) {
        cascadeMutex.lock();

        cv::Mat imageBW(storedImageBW.size().y,
                        storedImageBW.size().x,
                        CV_8UC1,
                        storedImageBW.data());

        cv::Mat image(storedImage.size().y,
                      storedImage.size().x,
                      CV_8UC3,
                      storedImage.data());

        cascadeMutex.unlock();

        cascades.push_back(cascade->checkCascade(imageBW));

        if (++frameCount == CASCADE_FRAMES)
            break;
    }

    if (cascades.size() != 0 && cascades[biggestArray].size() != 0) {

        for (unsigned int i = 0; i < cascades.size(); i++) {
           if (cascades[i].size() > cascades[biggestArray].size())
              biggestArray = i;
        }

        std::cout << "The biggest array is nr. " << biggestArray << std::endl;
        std::cout << "x: " << cascades[biggestArray][0].x << std::endl;
        std::cout << "y: " << cascades[biggestArray][0].y << std::endl;
    }


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
