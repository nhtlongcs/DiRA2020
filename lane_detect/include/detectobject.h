#ifndef DETECT_OBJECT_H
#define DETECT_OBJECT_H

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <list>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include "lane_detect/detectobjectConfig.h"

class DetectLane;

class DetectObject
{
public:
    DetectObject(DetectLane* lane);
    ~DetectObject();
    void updateDepth(const cv::Mat& depth);
    void updateBinary(const cv::Mat& binary);
    int detect();

public:
    void configCallback(lane_detect::detectobjectConfig& config, uint32_t level);
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

    ros::NodeHandle _nh;
    dynamic_reconfigure::Server<lane_detect::detectobjectConfig> _serverConfig;

    cv::Mat depth;
    cv::Mat binary;
    cv::Rect objectROIRect = {115, 100, 61, 49};

    int strategy = 0;

    // int detectThreshold;
    int diffDirectPercent = 5; // percents
    cv::Ptr<cv::BackgroundSubtractor> pBackSub;

    std::list<bool> objectHistories;
    int maxHistory = 10;
    int direct = 0;

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
    int depthThresholdMin = 10;
    int depthThresholdMax = 155;

    // Strategy getDirectOnKmean
    int kCluster = 3;

    image_transport::ImageTransport _debugImage;
    image_transport::Publisher _houghPublisher;
    image_transport::Publisher _depthThresholdedPublisher;
};

#endif