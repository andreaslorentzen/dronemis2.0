//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"
#include "Color.h"
#include "QR.h"

#define CASCADE_FRAMES 10
//#define DEBUG 1
#define DEBUG_COUT 1
//#define DEBUG_RED 1
//#define DEBUG_GREEN 1
#define DEBUG_BOX 1

Cascade *cascade;
Color *color;
Nav *navData;

using namespace std;
using namespace cv;

CV_Handler::CV_Handler(void) {

}


void CV_Handler::run(Nav *nav) {
    imageReady = false;
    navData = nav;
    frontCamSelected = true;
    graySelected = false;
    cascade = new Cascade();
    color = new Color();
    video_channel = nodeHandle.resolveName("ardrone/image_raw");
    video_subscriber = nodeHandle.subscribe(video_channel,10, &CV_Handler::video, this);
    namedWindow("BoxMis", CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("Thresh", "BoxMis", &thresh, 255); //Hue (0 - 255)
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

    imageReady = true;

    cascadeMutex.unlock();

    show();
}

void CV_Handler::show(void) {
    cv::Mat image;
    while(!imageReady);

    if (graySelected) {
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
#ifdef DEBUG_RED
        image = color->checkColorsRed(std::vector<Cascade::cubeInfo>(), image);
#elif DEBUG_GREEN
        image = color->checkColorsRed(std::vector<Cascade::cubeInfo>(), image);
#endif
#ifdef DEBUG_BOX
        image = checkBox(CV_Handler::boxCordsStruct());
#endif
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
    std::vector<cascadeArray> cascades;

    while (true) {
        while (!imageReady);
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
        if (!cascades[frameCount].empty())
            color->checkColors(cascades[frameCount],image);

        if (++frameCount == CASCADE_FRAMES)
            break;
    }
    if (!cascades.empty()) {
        for (unsigned int i = 0; i < cascades.size(); i++) {
           if (!cascades[i].empty() && cascades[i].size() > cascades[biggestArray].size())
              biggestArray = i;
        }
        calculatePosition(cascades[biggestArray]);

#ifdef DEBUG_COUT
        std::cout << "The biggest array is Nr. " << biggestArray << std::endl;
        std::cout << "x: " << cascades[biggestArray][0].x << std::endl;
        std::cout << "xDist: " << cascades[biggestArray][0].xDist << std::endl;
        std::cout << "y: " << cascades[biggestArray][0].y << std::endl;
        std::cout << "yDist: " << cascades[biggestArray][0].yDist << std::endl;
#endif
    }
    return std::vector<Cascade::cubeInfo>();
}

std::vector<Cascade::cubeInfo> CV_Handler::calculatePosition(std::vector<Cascade::cubeInfo> cubes) {
    double xFactor = 95.8 / 640;
    double yFactor = 51.7 / 360;

    for (unsigned int i = 0; i < cubes.size(); i++) {
        cubes[i].xDist = xFactor / navData->getPosition().z;
        cubes[i].yDist = yFactor / navData->getPosition().z;
    }

    return cubes;
}

 cv::Mat CV_Handler::checkBox(CV_Handler::boxCordsStruct boxcords) {
    Mat imgModded;
    std::vector<cv::Vec3f> circles;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    typedef std::vector<CV_Handler::boxCordsStruct> boxCordsArrayStruct;
    std::vector<boxCordsArrayStruct> boxCordsArray;
    Scalar color = Scalar((255), (255), (255));

    while (!imageReady);
    cascadeMutex.lock();
    cv::Mat imageBW(storedImageBW.size().y,
                    storedImageBW.size().x,
                    CV_8UC1,
                    storedImageBW.data());
#ifndef DEBUG
    cascadeMutex.unlock();
#endif
    // Add gaussian blur
    blur(imageBW, imgModded, Size(3,3));

    // Detect edges using canny
    Canny(imgModded, imgModded, thresh, thresh*2, 3);

    // Find contours
    findContours(imgModded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Draw contours
    for(unsigned int i = 0; i< contours.size(); i++)
        drawContours(imgModded, contours, i, color, 2, 8, hierarchy, 0, Point());

    // Detect circles
    cv::HoughCircles(imgModded, circles, CV_HOUGH_GRADIENT, 1, imgModded.rows/8, 100, 40, 10, 250);

    // Draw circles
    for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
        Point center( circles[current_circle][0], circles[current_circle][1]);
        boxcords.x = center.x;
        boxcords.y = center.y;


#ifdef DEBUG_COUT
        cout << "Found circle: " << center << ", radius: " << circles[current_circle][2] << endl;
#endif

#ifdef DEBUG_BOX_PAINT
        circle(imageBW, center, circles[current_circle][2], Scalar(0,0,255), 3, 8, 0 );
        return imageBW;
#endif
     }
     return imgModded;
}
