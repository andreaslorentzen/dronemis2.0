//
// Created by hippomormor on 6/4/16.
//

#include "Color.h"

using namespace cv;
using namespace std;

struct {
    int state;
    int kernel;
} data;

int kernelState = 0;
cv::Mat kernel;
void setKernel(int state, void* userdata);

Color::Color() {
    namedWindow("RedMis", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    cvCreateTrackbar("LowH", "RedMis", &(redFilter).iLowH, 255); //Hue (0 - 255)
    cvCreateTrackbar("HighH", "RedMis", &(redFilter).iHighH, 255);

    cvCreateTrackbar("LowS", "RedMis", &(redFilter).iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "RedMis", &(redFilter).iHighS, 255);

    cvCreateTrackbar("LowV", "RedMis", &(redFilter).iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "RedMis", &(redFilter).iHighV, 255);

    namedWindow("GreenMis", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    cvCreateTrackbar("LowH", "GreenMis", &(greenFilter).iLowH, 255); //Hue (0 - 255)
    cvCreateTrackbar("HighH", "GreenMis", &(greenFilter).iHighH, 255);

    cvCreateTrackbar("LowS", "GreenMis", &(greenFilter).iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "GreenMis", &(greenFilter).iHighS, 255);

    cvCreateTrackbar("LowV", "GreenMis", &(greenFilter).iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "GreenMis", &(greenFilter).iHighV, 255);

    cvCreateButton("Kernel",setKernel, &data,CV_PUSH_BUTTON,0);
}

Color::~Color() {

}

void setKernel(int state, void* userdata) {

    switch (kernelState++) {
        case 0 :
            kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
            cout << "kernel changed to ELLIPSE" << endl;
            break;
        case 1:
            kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            cout << "kernel changed to RECT" << endl;
            break;
        case 2:
            kernel = getStructuringElement(MORPH_CROSS, Size(5, 5));
            cout << "kernel changed to CROSS" << endl;
            kernelState = 0;
            break;
    }
}

std::vector<Cascade::cubeInfo> Color::checkColors(std::vector<Cascade::cubeInfo> cubes, cv::Mat image) {
    return checkColorsRed(checkColorsGreen(cubes,image),image);
}


std::vector<Cascade::cubeInfo> Color::checkColorsRed(std::vector<Cascade::cubeInfo> cubes, cv::Mat image) {

    Mat imgHSV;
    Mat nonZeros;
    Mat imgThresholded;

    //Convert from BGR to HSV
    cvtColor(image, imgHSV, COLOR_BGR2HSV);

    //morphological opening (remove small objects from the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,kernel);

    //Threshold the image to match filter-values
    inRange(imgHSV, Scalar(redFilter.iLowH, redFilter.iLowS, redFilter.iLowV), Scalar(redFilter.iHighH, redFilter.iHighS, redFilter.iHighV), imgThresholded);

    //morphological closing (fill small holes in the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,kernel);

    findNonZero(imgThresholded, nonZeros);

    //int count;
    for (unsigned int i = 0; i < cubes.size(); i++) {
        for (unsigned int j = 0; j < nonZeros.total(); j++) {
            if (nonZeros.at<Point>(j).x == cubes[i].x && nonZeros.at<Point>(j).y == cubes[i].y)
                cubes[i].color = 'r';
        }
    }
    
    return cubes;
}

std::vector<Cascade::cubeInfo> Color::checkColorsGreen(std::vector<Cascade::cubeInfo> cubes, cv::Mat image) {

    Mat imgHSV;
    Mat nonZeros;
    Mat imgThresholded;

    //Convert from BGR to HSV
    cvtColor(image, imgHSV, COLOR_BGR2HSV);

    //morphological opening (remove small objects from the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,kernel);

    //Threshold the image to match filter-values
    inRange(imgHSV, Scalar(greenFilter.iLowH, greenFilter.iLowS, greenFilter.iLowV), Scalar(greenFilter.iHighH, greenFilter.iHighS, greenFilter.iHighV), imgThresholded);

    //morphological closing (fill small holes in the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,kernel);

    findNonZero(imgThresholded, nonZeros);

    //int count;
    for (unsigned int i = 0; i < cubes.size(); i++) {
        for (unsigned int j = 0; j < nonZeros.total(); j++) {
            if (nonZeros.at<Point>(j).x == cubes[i].x && nonZeros.at<Point>(j).y == cubes[i].y)
                cubes[i].color = 'r';
        }
    }

    return cubes;
}