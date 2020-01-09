#ifndef SIGN_DETECT_H
#define SIGN_DETECT_H

#include <opencv2/opencv.hpp>
#include <list>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cds_msgs/sign.h>
#include <dynamic_reconfigure/server.h>
#include "sign_detect/SignConfig.h"

class SignDetect
{
public:
    SignDetect();
    ~SignDetect();
    
    void update();
    int detect();
public:
    void configCallback(sign_detect::SignConfig &config, uint32_t level);
    void updateRGBCallback(const sensor_msgs::ImageConstPtr &msg);
    void updateDepthCallback(const sensor_msgs::ImageConstPtr &msg);
    
private:
    int detectOneFrame();
    int classify(const cv::Mat &colorROI) const;
    int classifyCountBlue(const cv::Mat &colorROI) const;
    int classifyTemplateMatching(const cv::Mat &colorROI) const;

private:
    cv::Mat rgb;
    cv::Mat depth;

    const int MAX_FRAME_COUNT = 3;
    std::list<int> recentDetects;
    int canny = 255;
    int votes = 22;
    int frameCount = 0;

    int low_minBlue[3] = {47, 18, 0};
    int low_maxBlue[3] = {180, 150, 255};

    int minBlue[3] = {100, 90, 35};
    // int minBlue[3] = {77, 10, 20};
    int maxBlue[3] = {170, 142, 60};
    // int maxBlue[3] = {170, 142, 110};

    int detectConfident = 30, classifyConfident = 70;
    int diffToClassify = 5;
    int classifyStrategy = 1; // 0 - TemplateMatching, 1 - countBlue

    cv::Mat LEFT_TEMPLATE, RIGHT_TEMPLATE;
    int MAX_DIFF;

private:
    dynamic_reconfigure::Server<sign_detect::SignConfig> _server;

    ros::NodeHandle _nh;
    ros::Publisher _signPub;
    image_transport::ImageTransport _itSub;
    image_transport::Subscriber _rgbSub;
    image_transport::Subscriber _depthSub;

    image_transport::ImageTransport _debugImage;
    image_transport::Publisher _roiPublisher;
    image_transport::Publisher _detectPublisher;
    image_transport::Publisher _thresholdedPublisher;
};

#endif