#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <memory>
#include "opencv2/core.hpp"
#include "lane_detect/laneConfig.h"

class LaneLine;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    void detect();
    void show(const cv::Point* drivePoint=nullptr) const;
    int whichLane(const cv::Mat& objectMask) const;
    void updateBinary(const cv::Mat& binary);
    void updateRGB(const cv::Mat& rgb);

    bool isAbleToTurn(int direct) const;

    int getLaneWidth() const;
    std::shared_ptr<LaneLine> getLeftLane() const;
    std::shared_ptr<LaneLine> getRightLane() const;

    cv::Mat birdviewTransform(cv::Mat inputImage, cv::Mat& resultM) const;

public:
    void configlaneCallback(lane_detect::laneConfig& config, uint32_t level);

private:
    bool isWrongLane() const;
    cv::Mat preprocess(const cv::Mat& src);
    cv::Mat shadow(const cv::Mat& src);
    cv::Mat morphological(const cv::Mat& img);
    cv::Mat ROI(const cv::Mat& src);

    void drawLine(float slope, float yintercept, cv::Mat& HoughTransform);
    cv::Point Hough(const cv::Mat& img, const cv::Mat& src);

    bool isNeedRedetect(cv::Point leftBegin, cv::Point rightBegin) const;

    std::shared_ptr<LaneLine> leftLane;
    std::shared_ptr<LaneLine> rightLane;

    cv::Mat binary;
    cv::Mat rgb;
    cv::Mat debug;
    cv::Mat birdview;
    cv::Mat birdviewTransformMatrix;

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};
    
    int minLaneInShadow[3] = {90, 35, 95};
    int maxLaneInShadow[3] = {180, 117, 158};

    int initLaneWidth = 50;

    int lowThreshold = 2;
    int votes = 60;
    int minLinlength = 60;
    int maxLineGap = 5;
    size_t sumLaneWidth = 0;
    size_t frameCount = 0;

    int usebirdview = 1;

    int offsetLeft = 100;
    int offsetRight = 100;

    int offsetX = 160;
    int offsetY = 180;
    int birdwidth = 320;
    int birdheight = 240;
    int skyline = 120;
};
#endif
