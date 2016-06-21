//
// Created by hippomormor on 6/4/16.
//

#include "CV_Handler.h"
#include "Color.h"
#include "QR.h"

#define CASCADE_FRAMES 0

Cascade *cascade;
Color *color;
Nav *navData;
int filterState = 0;

struct buttonData{
    int state;
    int kernel;
    CV_Handler *cv;
}data;

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
    frontCamSelected = true;
    cascade = new Cascade();
    color = new Color();
    video_channel = nodeHandle.resolveName("ardrone/image_raw");
    video_subscriber = nodeHandle.subscribe(video_channel,10, &CV_Handler::video, this);
    namedWindow("MapMis", WINDOW_NORMAL);
    namedWindow("FilterMis", CV_WINDOW_AUTOSIZE);
    namedWindow("VideoMis", WINDOW_NORMAL);

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
    data.cv = this;
    cvCreateButton("Kernel", setKernel, &data,CV_PUSH_BUTTON,0);
    cvCreateButton("Filter", setFilter, &data,CV_PUSH_BUTTON,0);

    map = imread(map_name, CV_LOAD_IMAGE_UNCHANGED);
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
    struct buttonData *data;
    data = (struct buttonData *) userdata;

    if (++filterState > 4)
        filterState = 0;
    if (filterState == 0) {
        ROS_INFO("Standard video-feed");
        graySelected = false;
        data->cv->swapCam(true);
    }
    if (filterState == 1) {
        ROS_INFO("Red video-feed");
        graySelected = false;
        data->cv->swapCam(false);
    }
    if (filterState == 2) {
        ROS_INFO("Green video-feed");
        graySelected = false;
        data->cv->swapCam(false);
    }
    if (filterState == 3) {
        ROS_INFO("Circle threshold video-feed");
        graySelected = true;
        data->cv->swapCam(true);
    }
    if (filterState == 4) {
        ROS_INFO("Circle painted video-feed");
        graySelected = true;
        data->cv->swapCam(true);
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
            image = checkBox();
    } else {
        // Convert CVD byte array to OpenCV matrix (use CV_8UC3 format - unsigned 8 bit BGR 3 channel)
        cv::Mat imageBGR(storedImage.size().y,
                         storedImage.size().x,
                         CV_8UC3,
                         storedImage.data());
        image = imageBGR;
        if (filterState == 1)
            image = color->TESTcheckColorsRed(image);
        else if (filterState == 2)
            image = color->TESTcheckColorsGreen(image);
    }
    if (!image.empty())
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

void CV_Handler::checkCubes(void) {
    int frameCount = 0;
   // int biggestArray = 0;
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

        if (!cascades[frameCount].empty()) {
            color->checkColors(&cascades[frameCount], image);
            calculatePosition(&cascades[frameCount]);
        }

        if (frameCount == CASCADE_FRAMES)
            break;
        frameCount++;
    }
/*
    if (!cascades.empty()) {
        for (unsigned int i = 0; i < cascades.size(); i++) {
           if (!cascades[i].empty() && cascades[i].size() > cascades[biggestArray].size())
              biggestArray = i;
        }
       // calculatePosition(&cascades[biggestArray]);

#ifdef DEBUG_CV_COUT_EXPLICIT
            std::cout << "x: " << cascades[biggestArray][0].x << std::endl;
            std::cout << "xDist: " << cascades[biggestArray][0].xDist << std::endl;
            std::cout << "y: " << cascades[biggestArray][0].y << std::endl;
            std::cout << "yDist: " << cascades[biggestArray][0].yDist << std::endl;
#endif
       // std::cout << "The biggest array is Nr. " << biggestArray << std::endl;
        //std::cout << "color: " << cascades[biggestArray][0].color << std::endl;
    }*/
}

void CV_Handler::calculatePosition(std::vector<Cascade::cubeInfo> *cubes) {
    double xFactor = 95.8 / 640;
    double yFactor = 51.7 / 360;


    for (unsigned int i = 0; i < cubes->size(); i++) {
        if ((*cubes)[i].color.size() != 0) {

            (*cubes)[i].heading = (int) navData->getRotation();

            double x = (*cubes)[i].x - 320;
            double y = 180 - (*cubes)[i].y;

            (*cubes)[i].xDist = (xFactor / (navData->getPosition().z / 10)) * x;
            (*cubes)[i].yDist = (yFactor / (navData->getPosition().z / 10)) * y;

            double temp_rotation = ((*cubes)[i].heading - 90) * (-1);

            if (temp_rotation >= 360)
                temp_rotation -= 360;

            temp_rotation = temp_rotation / 180 * M_PI;

            Vector3 rotationMatrix[3];
            rotationMatrix[0] = Vector3(cos(temp_rotation), -sin(temp_rotation), 0);
            rotationMatrix[1] = Vector3(sin(temp_rotation), cos(temp_rotation), 0);
            rotationMatrix[2] = Vector3(0, 0, 1);

            Vector3 cubes_vector((*cubes)[i].xDist, (*cubes)[i].yDist, 0);
            Vector3 position_vector(navData->getPosition().x, navData->getPosition().y, 0);

            Vector3 resultVector(rotationMatrix[0].x * cubes_vector.x + rotationMatrix[0].y * cubes_vector.y +
                                 rotationMatrix[0].z * cubes_vector.z,
                                 rotationMatrix[1].x * cubes_vector.x + rotationMatrix[1].y * cubes_vector.y +
                                 rotationMatrix[1].z * cubes_vector.z,
                                 rotationMatrix[2].x * cubes_vector.x + rotationMatrix[2].y * cubes_vector.y +
                                 rotationMatrix[2].z * cubes_vector.z);

            resultVector.x += position_vector.x;
            resultVector.y += position_vector.y;

            bool isOK = true;
            if (!plottedCubes.empty()) {
                for (unsigned int i = 0; i < plottedCubes.size(); i++) {
                    if (abs(plottedCubes[i].x - resultVector.x) <= 20 && abs(plottedCubes[i].y - resultVector.y) <= 20) {
                        ROS_INFO("Cube discarded: Duplicate found");
                        isOK = false;
                    }
                }
                if (isOK) {
                    plottedCube cube;
                    cube.x = resultVector.x;
                    cube.y = resultVector.y;
                    plottedCubes.push_back(cube);
                    std::cout << "Color: " << (*cubes)[i].color << std::endl;
                    paintCube(Point(resultVector.x / 10, resultVector.y / 10), (*cubes)[i].color);
                }
            } else {
                plottedCube cube;
                cube.x = resultVector.x;
                cube.y = resultVector.y;
                plottedCubes.push_back(cube);
                std::cout << "Color: " << (*cubes)[i].color << std::endl;
                paintCube(Point(resultVector.x / 10, resultVector.y / 10), (*cubes)[i].color);
            }
        } else
            ROS_INFO("Cube discarded: Missing color");
    }
}

cv::Mat CV_Handler::checkBox(void) {
    Mat imgModded;
    std::vector<cv::Vec3f> circles;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Scalar color = Scalar((255), (255), (255));
    int median = 0;

    while (!imageReady);
    cascadeMutex.lock();
    cv::Mat imageBW(storedImageBW.size().y,
                    storedImageBW.size().x,
                    CV_8UC1,
                    storedImageBW.data());
    cascadeMutex.unlock();

    // Add gaussian blur
    blur(imageBW, imgModded, Size(3,3));

    int erosion_size = 6;
    Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                        cv::Point(erosion_size, erosion_size));

    // Apply erosion or dilation on the image
    dilate(imageBW,imageBW,element);

    // Detect edges using canny
    Canny(imgModded, imgModded, thresh, thresh*2, 3);

    // Find contours
    findContours(imgModded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Draw contours
    for(unsigned int i = 0; i< contours.size(); i++)
        drawContours(imgModded, contours, i, color, 2, 8, hierarchy, 0, Point());

    // Detect circles
    cv::HoughCircles(imgModded, circles, CV_HOUGH_GRADIENT, 1, imgModded.rows/8, 100, 55, 30, 200);

    if (!circles.empty()) {
        missingBoxFrames = 0;
        Point center(circles[0][0], circles[0][1]);
        boxVector.push_back((int) circles[0][2]);

        if (boxVector.size() == 20) {
            median = (int) findMedian(boxVector);
#ifdef DEBUG_CV_COUT
            cout << median << endl;
#endif
            boxVector.clear();
        }
        if (filterState == 4)
            circle(imageBW, center, circles[0][2], Scalar(0, 0, 255), 3, 8, 0);
    } else if (++missingBoxFrames == 10) {
        missingBoxFrames = 0;
        boxVector.clear();
    }
    if (median >= 60) {         // From 45
#ifdef DEBUG_CV_COUT
        cout << "Box is too close!" << endl;
#endif
    //                                              TODO Handle it!!
    }
    if (filterState == 3)
        return imgModded;

    return imageBW;
}

double CV_Handler::findMedian(std::vector<int> vec) {
    typedef vector<int>::size_type vec_sz;

    vec_sz size = vec.size();
    if (size == 0)
        return double();

    sort(vec.begin(), vec.end());

    vec_sz mid = size/2;

    return size % 2 == 0 ? (vec[mid] + vec[mid-1]) / 2 : vec[mid];
}

void CV_Handler::paintCube(Point center, std::string type) {
    int thickness = -1;
    int lineType = 8;
    if (!type.compare("Green")) {
        circle(map, cv::Point(navData->getPosition().x/10, navData->getPosition().y/10), 7, Scalar(0, 255, 0), thickness, lineType);
        imwrite(output_map_name, map);
    }
    else if (!type.compare("Red")){
        circle(map, cv::Point(navData->getPosition().x/10, navData->getPosition().y/10), 7, Scalar(0, 0, 255), thickness, lineType);
        imwrite(output_map_name, map);
    }
    imshow("MapMis", map);
    waitKey(10);
}