#ifndef DETECT_OBJECT_H
#define DETECT_OBJECT_H

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <list>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include "object_detect/ObjectDetectConfig.h"

class ObjectDetect
{
public:
    ObjectDetect();
    ~ObjectDetect();
    
    void update();

public:
    void configCallback(object_detect::ObjectDetectConfig &config, uint32_t level);
    void updateDepthCallback(const sensor_msgs::ImageConstPtr &msg);
    void updateBinaryCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    int estimateDirect(const cv::Mat &binaryROI);
    int getDirect(const cv::Rect &objectROI);
    int getDirectOnRawBinary(const cv::Rect &objectROI);
    int getDirectOnKmean(const cv::Rect &objectROI);
    int getDirectOnKmeanBGSub(const cv::Rect &objectROI);

    int detectOneFrame();
    void Hough(const cv::Mat &binary);
    void drawLine(float slope, float y_intercept, cv::Mat &HoughTransform);

private:
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

    int offsetX = 160;
    int offsetY = 180;
    int votes = 60;
    int canny_sign = 255;
    int votes_sign = 22;
    int minLinlength = 60;
    int maxLineGap = 5;

    // Strategy getDirectOnRawBinary
    int depthThresholdMin = 10;
    int depthThresholdMax = 155;

    // Strategy getDirectOnKmean
    int kCluster = 3;

    ros::NodeHandle _nh;
    ros::Publisher _objPub;
    dynamic_reconfigure::Server<object_detect::ObjectDetectConfig> _serverConfig;

    image_transport::ImageTransport _it;
    image_transport::Subscriber _depthSub;
    image_transport::Subscriber _binarySub;
};

#endif