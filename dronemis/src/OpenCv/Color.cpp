//
// Created by hippomormor on 6/4/16.
//

#include "Color.h"

using namespace cv;
using namespace std;

Color::Color() {
    kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
}

Color::~Color() {

}

std::vector<Cascade::cubeInfo> Color::checkColors(std::vector<Cascade::cubeInfo> *cubes, cv::Mat image) {
    checkColorsGreen(cubes,image);
    checkColorsRed(cubes,image);

    return std::vector<Cascade::cubeInfo>();
}


cv::Mat Color::checkColorsRed(std::vector<Cascade::cubeInfo> * cubes, cv::Mat image) {

    Mat imgHSV;
    Mat nonZeros;
    Mat imgThresholded;

    //Convert from BGR to HSV
    cvtColor(image, imgHSV, COLOR_BGR2HSV);

    //Threshold the image to match filter-values
    inRange(imgHSV, Scalar(redFilter.iLowH, redFilter.iLowS, redFilter.iLowV), Scalar(redFilter.iHighH, redFilter.iHighS, redFilter.iHighV), imgThresholded);

    //Morphological opening (remove small objects from the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,kernel);

    //Morphological closing (fill small holes in the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,kernel);

    //Find white areas
    findNonZero(imgThresholded, nonZeros);

    for (unsigned int j = 0; j < nonZeros.total(); j++) {
        for (unsigned int i = 0; i < cubes->size(); i++) {
            for (int h = -5; h < 5; h++) {
                if (nonZeros.at<Point>(j).x == (*cubes)[i].x + h && nonZeros.at<Point>(j).y == (*cubes)[i].y) {
                    (*cubes)[i].found++;
                }
                if (nonZeros.at<Point>(j).x == (*cubes)[i].x + h && nonZeros.at<Point>(j).y == (*cubes)[i].y+h) {
                    (*cubes)[i].found++;
                }
            }
            if ((*cubes)[i].found >= 2 && (*cubes)[i].color.size() == 0) {
                (*cubes)[i].color = "Red";
            }
        }
    }

    return imgThresholded;
}

cv::Mat Color::checkColorsGreen(std::vector<Cascade::cubeInfo> * cubes, cv::Mat image) {

    Mat imgHSV;
    Mat nonZeros;
    Mat imgThresholded;

    //Convert from BGR to HSV
    cvtColor(image, imgHSV, COLOR_BGR2HSV);

    //Threshold the image to match filter-values
    inRange(imgHSV, Scalar(greenFilter.iLowH, greenFilter.iLowS, greenFilter.iLowV), Scalar(greenFilter.iHighH, greenFilter.iHighS, greenFilter.iHighV), imgThresholded);

    //Morphological opening (remove small objects from the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,kernel);

    //Morphological closing (fill small holes in the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,kernel);

    //Find white areas
    findNonZero(imgThresholded, nonZeros);

    for (unsigned int j = 0; j < nonZeros.total(); j++) {
        for (unsigned int i = 0; i < cubes->size(); i++) {
            for (int h = -5; h < 5; h++) {
                if (nonZeros.at<Point>(j).x == (*cubes)[i].x + h && nonZeros.at<Point>(j).y == (*cubes)[i].y) {
                    (*cubes)[i].found++;
                }
                if (nonZeros.at<Point>(j).x == (*cubes)[i].x + h && nonZeros.at<Point>(j).y == (*cubes)[i].y+h) {
                    (*cubes)[i].found++;
                }
            }
            if ((*cubes)[i].found >= 2 && (*cubes)[i].color.size() == 0) {
                (*cubes)[i].color = "Green";
            }
        }
    }

    return imgThresholded;
}

cv::Mat Color::TESTcheckColorsGreen(cv::Mat image) {

    Mat imgHSV;
    Mat imgThresholded;

    //Convert from BGR to HSV
    cvtColor(image, imgHSV, COLOR_BGR2HSV);

    //Threshold the image to match filter-values
    inRange(imgHSV, Scalar(greenFilter.iLowH, greenFilter.iLowS, greenFilter.iLowV), Scalar(greenFilter.iHighH, greenFilter.iHighS, greenFilter.iHighV), imgThresholded);

    //Morphological opening (remove small objects from the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,kernel);

    //Morphological closing (fill small holes in the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,kernel);

    return imgThresholded;
}

cv::Mat Color::TESTcheckColorsRed(cv::Mat image) {

    Mat imgHSV;
    Mat imgThresholded;

    //Convert from BGR to HSV
    cvtColor(image, imgHSV, COLOR_BGR2HSV);

    //Threshold the image to match filter-values
    inRange(imgHSV, Scalar(redFilter.iLowH, redFilter.iLowS, redFilter.iLowV), Scalar(redFilter.iHighH, redFilter.iHighS, redFilter.iHighV), imgThresholded);

    //Morphological opening (remove small objects from the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,kernel);

    //Morphological closing (fill small holes in the foreground)
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,kernel);

    return imgThresholded;
}