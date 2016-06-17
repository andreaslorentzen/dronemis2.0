//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"
#include "Color.h"
#include "QR.h"

#define CASCADE_FRAMES 10

Cascade *cascade;
Color *color;
Nav *navData;
int filterState = 0;

using namespace std;
using namespace cv;
bool frontCamSelected = true;
bool graySelected = false;

void setKernel(int state, void* userdata);
void setFilter(int state, void* userdata);

CV_Handler::CV_Handler(void) {

}

void CV_Handler::run(Nav *nav) {
    imageReady = false;
    navData = nav;

    cascade = new Cascade();
    color = new Color();
    video_channel = nodeHandle.resolveName("ardrone/image_raw");
    video_subscriber = nodeHandle.subscribe(video_channel,10, &CV_Handler::video, this);
    namedWindow("FilterMis", CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("Thresh", "FilterMis", &thresh, 255); //Hue (0 - 255)

    cvCreateTrackbar("LowH_R", "FilterMis", &(color->redFilter).iLowH, 255); //Hue (0 - 255)
    cvCreateTrackbar("HighH_R", "FilterMis", &(color->redFilter).iHighH, 255);
    cvCreateTrackbar("LowS_R", "FilterMis", &(color->redFilter).iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS_R", "FilterMis", &(color->redFilter).iHighS, 255);
    cvCreateTrackbar("LowV_R", "FilterMis", &(color->redFilter).iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV_R", "FilterMis", &(color->redFilter).iHighV, 255);

    cvCreateTrackbar("LowH_G", "FilterMis", &(color->greenFilter).iLowH, 255); //Hue (0 - 255)
    cvCreateTrackbar("HighH_G", "FilterMis", &(color->greenFilter).iHighH, 255);
    cvCreateTrackbar("LowS_G", "FilterMis", &(color->greenFilter).iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS_G", "FilterMis", &(color->greenFilter).iHighS, 255);
    cvCreateTrackbar("LowV_G", "FilterMis", &(color->greenFilter).iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV_G", "FilterMis", &(color->greenFilter).iHighV, 255);

    cvCreateButton("Kernel", setKernel, &color->data,CV_PUSH_BUTTON,0);
    cvCreateButton("Filter", setFilter, &color->data,CV_PUSH_BUTTON,0);

    ros::Rate r(25);
    while(nodeHandle.ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

void setKernel(int state, void* userdata) {

    switch (color->kernelState++) {
        case 0 :
            color->kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
            cout << "kernel changed to ELLIPSE" << endl;
            break;
        case 1:
            color->kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            cout << "kernel changed to RECT" << endl;
            break;
        case 2:
            color->kernel = getStructuringElement(MORPH_CROSS, Size(5, 5));
            cout << "kernel changed to CROSS" << endl;
            color->kernelState = 0;
            break;
    }
}

void setFilter(int state, void* userdata) {
    if (++filterState > 4)
        filterState = 0;
    if (filterState == 0) {
        ROS_INFO("Standard video-feed");
        graySelected = false;
    }
    if (filterState == 1) {
        ROS_INFO("Red video-feed");
        graySelected = false;
    }
    if (filterState == 2) {
        ROS_INFO("Green video-feed");
        graySelected = false;
    }
    if (filterState == 3) {
        ROS_INFO("Circle threshold video-feed");
        graySelected = true;
    }
    if (filterState == 4) {
        ROS_INFO("Circle painted video-feed");
        graySelected = true;
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
    if (filterState == 3 || filterState == 4)
        image = checkBox(CV_Handler::boxCordsStruct());
    } else {
        // Convert CVD byte array to OpenCV matrix (use CV_8UC3 format - unsigned 8 bit BGR 3 channel)
        cv::Mat imageBGR(storedImage.size().y,
                         storedImage.size().x,
                         CV_8UC3,
                 storedImage.data());
        image = imageBGR;
        if (filterState == 1)
            image = color->checkColorsRed(std::vector<Cascade::cubeInfo>(), image);
        else if (filterState == 2)
            image = color->checkColorsGreen(std::vector<Cascade::cubeInfo>(), image);
        else if (filterState == 3)
            image = checkBox(CV_Handler::boxCordsStruct());
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

#ifdef DEBUG_CV_COUT
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
    cascadeMutex.unlock();

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
        Point center(circles[current_circle][0], circles[current_circle][1]);
        boxcords.x = center.x;
        boxcords.y = center.y;
        boxcords.radius = circles[current_circle][2];
        if(boxcords.radius >=45) boxcords.boxIsTooClose = 1;
        else boxcords.boxIsTooClose = 0;

        cout << "Boxistooclose = " << boxcords.boxIsTooClose << endl;

#ifdef DEBUG_CV_COUT
        cout << "Found circle: " << center << ", radius: " << circles[current_circle][2] << endl;
#endif
        if (filterState == 4)
            circle(imageBW, center, circles[current_circle][2], Scalar(0, 0, 255), 3, 8, 0);
        return imageBW;
     }
     if (filterState == 3)
         return imgModded;
     else if (filterState == 4)
         return imageBW;

     return cv::Mat();
}