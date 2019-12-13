#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "detectobject.h"
#include "utils.h"

DetectObject::DetectObject()
: objectROI{115, 62, 61, 49}
, kCluster{3}
, pBackSub{cv::createBackgroundSubtractorMOG2()}
, detectThreshold{30}
, configObjectmin{0, 0, 54}
, configObjectmax{255, 205, 163}
{
    cv::namedWindow("ConfigDetectObject", cv::WINDOW_GUI_NORMAL);
    cv::createTrackbar("clusterCount", "ConfigDetectObject", &kCluster, 10);
    

    cv::createTrackbar("x", "ConfigDetectObject", &objectROI.x, 320);
    cv::createTrackbar("y", "ConfigDetectObject", &objectROI.y, 240);
    cv::createTrackbar("width", "ConfigDetectObject", &objectROI.width, 320);
    cv::createTrackbar("height", "ConfigDetectObject", &objectROI.height, 240);

    cv::createTrackbar("Detect threshold", "ConfigDetectObject", &detectThreshold, 100); // percents

    cv::createTrackbar("min H", "ConfigDetectObject", &configObjectmin[0], 255);
    cv::createTrackbar("max H", "ConfigDetectObject", &configObjectmax[0], 255);
    cv::createTrackbar("min S", "ConfigDetectObject", &configObjectmin[1], 255);
    cv::createTrackbar("max S", "ConfigDetectObject", &configObjectmax[1], 255);
    cv::createTrackbar("min V", "ConfigDetectObject", &configObjectmin[2], 255);
    cv::createTrackbar("max V", "ConfigDetectObject", &configObjectmax[2], 255);
}

void DetectObject::update(const cv::Mat& depth)
{
    // cv::cvtColor(depth, this->depth, cv::COLOR_BGR2GRAY);
    this->depth = depth.clone();
}

bool DetectObject::detect()
{
    if (this->depth.empty())
    {
        return false;
    }
    cv::rectangle(this->depth, objectROI, cv::Scalar{0, 0, 255}, 2);
    cv::imshow("DepthObjectROI", this->depth);

    cv::Mat objectROIImage = this->depth(objectROI);
    cv::Mat kmeanImage = kmean(objectROIImage, kCluster);

    // {
    //     double minVal, maxVal;
    //     cv::minMaxLoc(kmeanImage, &minVal, &maxVal, NULL, NULL);
    //     int minValInt = cvRound(minVal), maxValInt = cvRound(maxVal);
    //     cv::Mat mask = (kmeanImage != maxValInt);
    //     cv::imshow("Mask", mask);
    // }
    
    cv::Mat hsv;
    cv::Mat mask;
    cv::cvtColor(kmeanImage, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(configObjectmin[0], configObjectmin[1], configObjectmin[2]),
                    cv::Scalar(configObjectmax[0], configObjectmax[1], configObjectmax[2]), mask);
    cv::imshow("cropped", mask);

    cv::Mat fgMask;
    pBackSub->apply(kmeanImage, fgMask);
    
    fgMask &= mask;
    cv::imshow("foreground", fgMask);

    float percent = cv::countNonZero(fgMask) * 100.0f/ (fgMask.rows * fgMask.cols);
    std::cout << "percent :" << percent << std::endl;
    if ( percent >= detectThreshold)
    {
        std::cout << "OBJECTTTTT" << std::endl;
        return true;
    }

    return false;
}