#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include "detectobject.h"
#include "detectlane.h"
#include "utils.h"

constexpr const char* CONF_OBJ_WINDOW = "ConfigDetectObject";

DetectObject::DetectObject(DetectLane* lane)
: objectROI{115, 62, 61, 49}
, kCluster{3}
, pBackSub{cv::createBackgroundSubtractorMOG2()}
, detectThreshold{30}
, configObjectmin{0, 0, 54}
, configObjectmax{255, 205, 163}
, lane{lane}
{
    cv::namedWindow(CONF_OBJ_WINDOW, cv::WINDOW_GUI_NORMAL);
    cv::createTrackbar("clusterCount", CONF_OBJ_WINDOW, &kCluster, 10);

    cv::createTrackbar("x", CONF_OBJ_WINDOW, &objectROI.x, 320);
    cv::createTrackbar("y", CONF_OBJ_WINDOW, &objectROI.y, 240);
    cv::createTrackbar("width", CONF_OBJ_WINDOW, &objectROI.width, 320);
    cv::createTrackbar("height", CONF_OBJ_WINDOW, &objectROI.height, 240);

    cv::createTrackbar("Detect threshold", CONF_OBJ_WINDOW, &detectThreshold, 100); // percents

    cv::createTrackbar("min H", CONF_OBJ_WINDOW, &configObjectmin[0], 255);
    cv::createTrackbar("max H", CONF_OBJ_WINDOW, &configObjectmax[0], 255);
    cv::createTrackbar("min S", CONF_OBJ_WINDOW, &configObjectmin[1], 255);
    cv::createTrackbar("max S", CONF_OBJ_WINDOW, &configObjectmax[1], 255);
    cv::createTrackbar("min V", CONF_OBJ_WINDOW, &configObjectmin[2], 255);
    cv::createTrackbar("max V", CONF_OBJ_WINDOW, &configObjectmax[2], 255);
}

DetectObject::~DetectObject()
{
    cv::destroyWindow(CONF_OBJ_WINDOW);
}

void DetectObject::update(const cv::Mat& depth)
{
    // cv::cvtColor(depth, this->depth, cv::COLOR_BGR2GRAY);
    this->depth = depth.clone();
}

// bool DetectObject::detect()
// {
//     if (this->depth.empty())
//     {
//         return false;
//     }
//     this->depth = birdviewTransformation(this->depth);
    
//     cv::rectangle(this->depth, objectROI, cv::Scalar{0, 0, 255}, 2);
//     cv::imshow("DepthObjectROI", this->depth);
//     cv::Mat objectROIImage = this->depth(objectROI);
//     cv::Mat kmeanImage = kmean(objectROIImage, kCluster);

//     // {
//     //     double minVal, maxVal;
//     //     cv::minMaxLoc(kmeanImage, &minVal, &maxVal, NULL, NULL);
//     //     int minValInt = cvRound(minVal), maxValInt = cvRound(maxVal);
//     //     cv::Mat mask = (kmeanImage != maxValInt);
//     //     cv::imshow("Mask", mask);
//     // }
    
//     cv::Mat hsv;
//     cv::Mat mask;
//     cv::cvtColor(kmeanImage, hsv, cv::COLOR_BGR2HSV);
//     cv::inRange(hsv, cv::Scalar(configObjectmin[0], configObjectmin[1], configObjectmin[2]),
//                     cv::Scalar(configObjectmax[0], configObjectmax[1], configObjectmax[2]), mask);
//     cv::imshow("cropped", mask);

//     cv::Mat fgMask;
//     pBackSub->apply(kmeanImage, fgMask);
    
//     fgMask &= mask;
//     cv::imshow("foreground", fgMask);

//     float percent = cv::countNonZero(fgMask) * 100.0f/ (fgMask.rows * fgMask.cols);
//     std::cout << "percent :" << percent << std::endl;
//     if ( percent >= detectThreshold)
//     {
//         std::cout << "OBJECTTTTT" << std::endl;
//         return true;
//     }

//     return false;
// }
// new detect
bool DetectObject::estimator(const cv::Mat& binaryROI){
    int W = binaryROI.size().width;
    int H = binaryROI.size().height;
    cv::Rect objectROI_left({0, 0, W/2, H});
    cv::Rect objectROI_right({W/2, 0, W/2, H});
    cv::Mat ImageLeft = binaryROI(objectROI_left);
    cv::Mat ImageRight = binaryROI(objectROI_right);
    return cv::countNonZero(ImageLeft) > cv::countNonZero(ImageRight) ? 1 : 0;

}

int DetectObject::getDirect() const
{
    return this->direct;
}

bool DetectObject::detectOneFrame()
{
    if (this->depth.empty())
    {
        return false;
    }

    // this->depth = birdviewTransformation(this->depth);
    
    cv::rectangle(this->depth, objectROI, cv::Scalar{0, 0, 255}, 2);
    cv::Mat objectROIImage = this->depth(objectROI);
    cv::Mat kmeanImage = kmean(objectROIImage, kCluster);

    // cv::imshow("DepthObjectROI", kmeanImage);
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
    cv::imshow(CONF_OBJ_WINDOW, mask);

    cv::Mat fgMask;
    pBackSub->apply(kmeanImage, fgMask);
    
    fgMask &= mask;
    cv::imshow("foreground", fgMask);

    float percent = cv::countNonZero(fgMask) * 100.0f/ (fgMask.rows * fgMask.cols);
    if ( percent >= detectThreshold)
    {
        // cv::Mat laneImagePerspective;
        // this->lane->show(laneImagePerspective);

        // laneImagePerspective |= fgMask;
        // cv::imshow("ObjectAndLane", laneImagePerspective);


        return true;
    }

    return false;
}

bool DetectObject::detect()
{
    bool object = detectOneFrame();
    objectHistories.push_back(object);
    if (objectHistories.size() > maxHistory)
    {
        objectHistories.pop_front();
    }

    int count = std::count(objectHistories.begin(), objectHistories.end(), true);
    if ((count*100.0f/maxHistory) > 50)
    {
        return true;
    }
    return false;
}