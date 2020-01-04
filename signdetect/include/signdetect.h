#ifndef DETECT_SIGN_H
#define DETECT_SIGN_H

#include <opencv2/core.hpp>
#include <list>
#include <image_transport/image_transport.h>
#include "signdetect/signConfig.h"

class DetectSign
{
public:
    DetectSign(const cv::Mat& leftTemplate, const cv::Mat& rightTemplate);
    ~DetectSign();
    void updateRGB(const cv::Mat &rgb);
    void updateDepth(const cv::Mat &depth);
    int detect();

    void configCallback(signdetect::signConfig &config, uint32_t level);

private:
    int detectOneFrame();
    int classify(const cv::Mat& colorROI) const;
    int classifyCountBlue(const cv::Mat& colorROI) const;
    int classifyTemplateMatching(const cv::Mat& colorROI) const;

private:
    cv::Mat rgb;
    cv::Mat depth;

    const int MAX_FRAME_COUNT = 3;
    std::list<int> recentDetects;
    int canny = 255;
    int votes = 22;
    int frameCount = 0;
    int minBlue[3] = {100, 90, 35};
    // int minBlue[3] = {77, 10, 20};
    int maxBlue[3] = {170, 142, 60};
    // int maxBlue[3] = {170, 142, 110};


    int detectConfident = 30, classifyConfident = 70;
    int diffToClassify = 5;
    int classifyStrategy = 1; // 0 - TemplateMatching, 1 - countBlue

    const cv::Mat LEFT_TEMPLATE, RIGHT_TEMPLATE;
    int MAX_DIFF;

    ros::NodeHandle _nh;
    image_transport::ImageTransport _debugImage;
    image_transport::Publisher _roiPublisher;
    image_transport::Publisher _detectPublisher;
    image_transport::Publisher _thresholdedPublisher;
};

#endif