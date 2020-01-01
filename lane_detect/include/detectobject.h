#ifndef DETECT_OBJECT_H
#define DETECT_OBJECT_H

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <list>

class DetectLane;

class DetectObject
{
public:
    DetectObject(DetectLane* lane);
    ~DetectObject();
    void updateDepth(const cv::Mat& depth);
    void updateBinary(const cv::Mat& binary);
    int detect();
private:
    int estimateDirect(const cv::Mat& binaryROI);
    int getDirect(const cv::Rect& objectROI);
    int getDirectOnRawBinary(const cv::Rect& objectROI);
    int getDirectOnKmean(const cv::Rect& objectROI);
    int getDirectOnKmeanBGSub(const cv::Rect& objectROI);

    int detectOneFrame();
    void Hough(const cv::Mat& binary);
    void drawLine(float slope, float y_intercept, cv::Mat &HoughTransform);
private:
    cv::Mat depth;
    cv::Mat binary;
    cv::Rect objectROIRect;

    int strategy = 0;

    int detectThreshold;
    int diffDirectPercent = 5; // percents
    cv::Ptr<cv::BackgroundSubtractor> pBackSub;

    std::list<bool> objectHistories;
    int maxHistory = 10;
    int direct;

    int offsetROI_x = 30;
    int offsetROI_y = 95;
    int objectROI_offsetTop = 45;

    DetectLane* lane;

    int offsetX = 160;
    int offsetY = 180;
    int votes = 60;
    int minLinlength = 60;
    int maxLineGap = 5;

    // Strategy getDirectOnRawBinary
    int depthThresholdMin = 0;
    int depthThresholdMax = 155;

    // Strategy getDirectOnKmean
    int kCluster;
};

#endif