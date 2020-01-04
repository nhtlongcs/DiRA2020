#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "detectobject.h"
#include "detectlane.h"
#include "utils.h"

constexpr const char* CONF_OBJ_WINDOW = "ConfigDetectObject";

enum OBJ_STRATEGY
{
    ON_RAW_BINARY = 0,
    ON_KMEAN,
    LEN
};

DetectObject::DetectObject(DetectLane* lane)
: pBackSub{cv::createBackgroundSubtractorMOG2()}
, lane{lane}
, _nh{"detectobject"}
, _serverConfig{_nh}
, _debugImage{_nh}
{
    _serverConfig.setCallback(boost::bind(&DetectObject::configCallback, this, _1, _2));
    _houghPublisher = _debugImage.advertise("/debug/object/hough", 1, false);
    _depthThresholdedPublisher = _debugImage.advertise("/debug/object/depth_threshold", 1, false);
    // cv::namedWindow(CONF_OBJ_WINDOW, cv::WINDOW_GUI_NORMAL);
    // cv::createTrackbar("clusterCount", CONF_OBJ_WINDOW, &kCluster, 10);

    // cv::createTrackbar("strategy", CONF_OBJ_WINDOW, &strategy, OBJ_STRATEGY::LEN - 1);

    // cv::createTrackbar("objectROI_x", CONF_OBJ_WINDOW, &offsetROI_x, 320);
    // cv::createTrackbar("objectROI_y", CONF_OBJ_WINDOW, &offsetROI_y, 240);
    // // cv::createTrackbar("objectROI_w", CONF_OBJ_WINDOW, &objectROIRect.width, 320);
    // cv::createTrackbar("objectROI_h", CONF_OBJ_WINDOW, &objectROIRect.height, 240);
    // cv::createTrackbar("objectROI_offsetTop", CONF_OBJ_WINDOW, &objectROI_offsetTop, 100);

    // cv::createTrackbar("Detect threshold", CONF_OBJ_WINDOW, &detectThreshold, 100); // percents
    // cv::createTrackbar("DiffToKnowLeftRight", CONF_OBJ_WINDOW, &diffDirectPercent, 100); // percents

    // cv::createTrackbar("DepthThresholdMin", CONF_OBJ_WINDOW, &depthThresholdMin, 255); // percents
    // cv::createTrackbar("DepthThresholdMax", CONF_OBJ_WINDOW, &depthThresholdMax, 255); // percents

    // cv::createTrackbar("min H", CONF_OBJ_WINDOW, &configObjectmin[0], 255);
    // cv::createTrackbar("max H", CONF_OBJ_WINDOW, &configObjectmax[0], 255);
    // cv::createTrackbar("min S", CONF_OBJ_WINDOW, &configObjectmin[1], 255);
    // cv::createTrackbar("max S", CONF_OBJ_WINDOW, &configObjectmax[1], 255);
    // cv::createTrackbar("min V", CONF_OBJ_WINDOW, &configObjectmin[2], 255);
    // cv::createTrackbar("max V", CONF_OBJ_WINDOW, &configObjectmax[2], 255);
}

DetectObject::~DetectObject()
{
    // cv::destroyWindow(CONF_OBJ_WINDOW);
}

void DetectObject::configCallback(lane_detect::detectobjectConfig& config, uint32_t level)
{
    kCluster = config.cluster_count;
    offsetROI_x = config.offsetROI_x;
    offsetROI_y = config.offsetROI_y;
    objectROIRect.height = config.objectROI_h;
    objectROI_offsetTop = config.objectROI_offsetTop;
    diffDirectPercent = config.diff_to_know_left_right;
    depthThresholdMin = config.depth_threshold_min;
    depthThresholdMax = config.depth_threshold_max;
    strategy = config.strategy;
}

void DetectObject::updateDepth(const cv::Mat& depth)
{
    this->depth = depth.clone();
}

void DetectObject::updateBinary(const cv::Mat& binary)
{
    this->binary = binary.clone();
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

int DetectObject::estimateDirect(const cv::Mat& binaryROI)
{
    if (binaryROI.rows < 10 || binaryROI.cols < 20)
    {
        // if ROI is too small
        return 0;
    }
    cv::Rect objectROI_left{0, 0, binaryROI.cols/2, binaryROI.rows};
    cv::Rect objectROI_right{binaryROI.cols/2, 0, binaryROI.cols/2, binaryROI.rows};

    cv::Mat ImageLeft = binaryROI(objectROI_left);
    cv::Mat ImageRight = binaryROI(objectROI_right);

    int countLeft = cv::countNonZero(ImageLeft);
    int countRight = cv::countNonZero(ImageRight);

    float percentLeft = countLeft * 100.0f / (ImageLeft.rows*ImageLeft.cols); // 0-100%
    float percentRight = countRight * 100.0f / (ImageRight.rows*ImageRight.cols); // 0-100%

    ROS_INFO("Diff Object = %.2f", percentLeft - percentRight);
    if (abs(percentLeft - percentRight) > diffDirectPercent)
    {
        return (percentLeft > percentRight) ? -1 : 1;
    }
    return 0;
}

int DetectObject::getDirect(const cv::Rect& objectROI)
{
    int result = 0;
    switch (strategy)
    {
        case 1: result = getDirectOnKmean(objectROI); break;
        case 2: result = getDirectOnKmeanBGSub(objectROI); break;

        default:
        case 0: result = getDirectOnRawBinary(objectROI); break;
    }
    return result;
}

int DetectObject::getDirectOnRawBinary(const cv::Rect& objectROI)
{
    cv::Mat depthThresholded;
    depthThresholded = this->depth(objectROI);
    cv::inRange(depthThresholded, cv::Scalar{depthThresholdMin*1.0}, cv::Scalar{depthThresholdMax*1.0}, depthThresholded);
    

    showImage(_depthThresholdedPublisher, "mono8", depthThresholded);
    return estimateDirect(depthThresholded);
}

int DetectObject::getDirectOnKmean(const cv::Rect& objectROI)
{
    // cv::Mat kmeanImage = kmean(this->depth, kCluster);
    // cv::imshow("KMean", kmeanImage);

    // cv::Mat objectROIImage = kmeanImage(objectROI);
    // cv::imshow("KMeanCrop", objectROIImage);

    // return 0;
    cv::Mat objectROIImage = this->depth(objectROI);
    cv::Mat kmeanImage = kmean(objectROIImage, kCluster);


    cv::Mat mask;
    cv::inRange(kmeanImage, cv::Scalar{depthThresholdMin*1.0}, cv::Scalar{depthThresholdMax*1.0}, mask);

    // cv::imshow(CONF_OBJ_WINDOW, mask);

    return estimateDirect(mask);
}

int DetectObject::getDirectOnKmeanBGSub(const cv::Rect& objectROI)
{
    cv::Mat depthThresholded;
    depthThresholded = this->depth(objectROI);
    cv::inRange(depthThresholded, cv::Scalar{depthThresholdMin*1.0}, cv::Scalar{depthThresholdMax*1.0}, depthThresholded);
    // showImage(CONF_OBJ_WINDOW, depthThresholded);
    cv::Mat fgMask;
    pBackSub->apply(this->depth, fgMask);
    int tmp = estimateDirect(depthThresholded);
    if (!tmp)
    {   
        fgMask = fgMask(objectROI);
        depthThresholded &= fgMask;
        tmp = estimateDirect(depthThresholded);
    } 

    showImage(_depthThresholdedPublisher, "mono8", depthThresholded);

    return tmp;
}
    
int DetectObject::detectOneFrame()
{
    if (this->binary.empty() || this->depth.empty())
    {
        return 0;
    }

    {
        using namespace std;
        using namespace cv;
        Mat gray;
        gray = kmean(this->depth, 2);

        // cvtColor(gray, gray, CV_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);

        vector<Vec3f> circles;

        HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
                    1,       // accumulator resolution (size of the image / 2)
                    300,     // minimum distance between two circles
                    canny_sign,   // Canny high threshold
                    votes_sign,   // minimum number of votes
                    0, 100); // min and max radius

        for (size_t i = 0; i < circles.size(); i++)
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            // circle(this->depth, center, 3, Scalar(127), -1, 8, 0);
            // // circle outline
            // circle(this->depth, center, radius, Scalar(100), 3, 8, 0);

            radius += 10;
            cv::Rect roi(center.x - radius, center.y - radius, radius * 2, radius * 2);
            

            if ((roi & cv::Rect(0, 0, gray.cols, gray.rows)) == roi)
            {
                this->depth(roi) = cv::Scalar{0};
            }
        }
    }


    Hough(this->binary);
    // ROS_INFO("objectROI = %d, %d, %d, %d", objectROIRect.x, objectROIRect.y, objectROIRect.width, objectROIRect.height);

    return getDirect(objectROIRect);

    // this->depth = birdviewTransformation(this->depth);
    
    // cv::rectangle(this->depth, objectROI, cv::Scalar{0, 0, 255}, 2);
    // cv::Mat objectROIImage = this->depth(objectROI);
    // cv::Mat kmeanImage = kmean(objectROIImage, kCluster);

    // cv::imshow("DepthObjectROI", kmeanImage);
    // {
    //     double minVal, maxVal;
    //     cv::minMaxLoc(kmeanImage, &minVal, &maxVal, NULL, NULL);
    //     int minValInt = cvRound(minVal), maxValInt = cvRound(maxVal);
    //     cv::Mat mask = (kmeanImage != maxValInt);
    //     cv::imshow("Mask", mask);
    // }
    
    // cv::Mat hsv;
    // cv::Mat mask;
    // cv::cvtColor(kmeanImage, hsv, cv::COLOR_BGR2HSV);
    // cv::inRange(hsv, cv::Scalar(configObjectmin[0], configObjectmin[1], configObjectmin[2]),
    //                 cv::Scalar(configObjectmax[0], configObjectmax[1], configObjectmax[2]), mask);
    // cv::imshow(CONF_OBJ_WINDOW, mask);

    // cv::Mat fgMask;
    // pBackSub->apply(kmeanImage, fgMask);
    
    // fgMask &= mask;
    // cv::imshow("foreground", fgMask);

    // float percent = cv::countNonZero(fgMask) * 100.0f/ (fgMask.rows * fgMask.cols);
    // if ( percent >= detectThreshold)
    // {
    //     // cv::Mat laneImagePerspective;
    //     // this->lane->show(laneImagePerspective);

    //     // laneImagePerspective |= fgMask;
    //     // cv::imshow("ObjectAndLane", laneImagePerspective);

    //     // int direct = this->lane->whichLane(objectROI);
    //     // ROS_INFO("Object on the %d", direct);

    //     return true;
    // }
}



void DetectObject::Hough(const cv::Mat& binary)
{
    std::vector<cv::Vec4f> lines;
    cv::Mat HoughTransform = cv::Mat::zeros(binary.size(), CV_8UC3);
    cv::Vec2f lineL{0.0f,0.0f}, lineR{0.0f,0.0f};
    int cntL = 0, cntR = 0;
    float leftSlope_average = 0.0;
    float rightSlope_average = 0.0;
    float leftYintercept_average = 0.0;
    float rightYintercept_average = 0.0;

    cv::HoughLinesP(binary, lines, 1, CV_PI / 180, votes, minLinlength, maxLineGap);

    for (const cv::Vec4f& line : lines)
    {
        float slope = 0;
        if (abs(line[0] - line[2]) < 0.01f)
        {
            // vertical line, tan = inf
            slope = 0;
            continue;
        } else
        {
            slope = (line[1] - line[3]) / (line[0] - line[2]);
        }
        float Yintercept = line[1] - slope * line[0];
        
        if (slope < 0) { 
            leftSlope_average += slope;
            leftYintercept_average += Yintercept;
            cntL++;
        }
        else {
            rightSlope_average += slope;
            rightYintercept_average += Yintercept;
            cntR++;
        }
    }

    if (cntL > 0) {
        leftSlope_average /= cntL;
        leftYintercept_average /= cntL;
        drawLine(leftSlope_average, leftYintercept_average, HoughTransform);
        lineL = cv::Vec2f{leftSlope_average, leftYintercept_average};
    }

    if (cntR > 0) {
        rightSlope_average /= cntR;
        rightYintercept_average /= cntR;
        drawLine(rightSlope_average, rightYintercept_average, HoughTransform);
        lineR = cv::Vec2f{rightSlope_average, rightYintercept_average};
    }

    float midY = offsetY;
    float midX = offsetX;

    if (fabs(rightSlope_average) < 0.05f) cntR = 0;
    if (fabs(leftSlope_average) < 0.05f) cntL = 0;
    
    if (cntL == 0 && cntR > 0) {
        midX = (midY - rightYintercept_average) / rightSlope_average;
    } 
    else if (cntL > 0 && cntR == 0) {
        midX = (midY - leftYintercept_average) / leftSlope_average;
    }
    else if (cntL > 0 && cntR > 0) {
        midX = (((midY - leftYintercept_average) / leftSlope_average) + ((midY - rightYintercept_average) / rightSlope_average)) / 2.0f;
    }

    cv::Mat colorBinary;

    cv::Mat depthThresholded = this->depth.clone();
    cv::inRange(depthThresholded, cv::Scalar{depthThresholdMin*1.0}, cv::Scalar{depthThresholdMax*1.0}, depthThresholded);

    cv::cvtColor(depthThresholded, colorBinary, cv::COLOR_GRAY2BGR);
    cv::addWeighted(colorBinary, 0.5, HoughTransform, 1, 1, HoughTransform);

    cv::circle(HoughTransform, cv::Point(midX, midY), 3, cv::Scalar(0, 0, 255), -1);
    cv::circle(HoughTransform, cv::Point(offsetX, offsetY), 3, cv::Scalar(0, 255, 0), -1);

    if ((lineL[0] != 0) && (lineR[0] != 0.0f))
    {
        int left_most_x, right_most_x;
        // ROS_INFO("lineL = [%.2f, %.2f], lineR = [%.2f, %.2f]", lineL[0], lineL[1], lineR[0], lineR[1]);

        left_most_x = (offsetROI_y - lineL[1]) / lineL[0];
        right_most_x = (offsetROI_y - lineR[1]) / lineR[0];

        if (left_most_x > right_most_x)
        {
            std::swap(left_most_x, right_most_x);
        }

        // ROS_INFO("1: left_most_x = %d, right_most_x = %d", left_most_x, right_most_x);

        left_most_x = std::min(std::max(left_most_x - offsetROI_x, 0), binary.cols - 1);
        right_most_x = std::min(std::max(right_most_x + offsetROI_x, 0), binary.cols - 1);

        // ROS_INFO("2: left_most_x = %d, right_most_x = %d", left_most_x, right_most_x);

        if (abs(left_most_x - right_most_x) <= 10)
        {
            return;
        }

        objectROIRect.x = left_most_x;
        objectROIRect.y = std::max(0, offsetROI_y - objectROI_offsetTop);
        objectROIRect.width = std::abs(right_most_x - left_most_x);
        cv::rectangle(HoughTransform, objectROIRect, cv::Scalar{255, 0, 0}, 2);

        // showImage("HoughLines", HoughTransform);
        showImage(_houghPublisher, "bgr8", HoughTransform);
    }
}

void DetectObject::drawLine(float slope, float y_intercept, cv::Mat &HoughTransform) {
    float y0 = 240.0;
    float y1 = 1.0;
    float x0 = (y0 - y_intercept) / slope;
    float x1 = (y1 - y_intercept) / slope;
    cv::line(HoughTransform, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

int DetectObject::detect()
{
    int object = detectOneFrame();
    return object;
    // objectHistories.push_back(object);
    // if (objectHistories.size() > maxHistory)
    // {
    //     objectHistories.pop_front();
    // }

    // int cntLeft = std::count(objectHistories.begin(), objectHistories.end(), -1);
    // int cntRight = std::count(objectHistories.begin(), objectHistories.end(), 1);
    // int cntNone = std::count(objectHistories.begin(), objectHistories.end(), 0);
    
    // int max = std::max(cntLeft, std::max(cntRight, cntNone));
    // if (max == cntLeft)
    //     return -1;
    // else if (max == cntRight)
    //     return 1;
    // else
    //     return 0;
}