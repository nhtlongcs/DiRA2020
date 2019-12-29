#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <memory>
#include "opencv2/core.hpp"

class LaneLine;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    void detect();
    void show(const cv::Point* drivePoint=nullptr) const;
    void updateBinary(const cv::Mat& binary);
    void updateDepth(const cv::Mat& depth);
    void updateRGB(const cv::Mat& rgb);

    int getLaneWidth() const;
    std::shared_ptr<LaneLine> getLeftLane() const;
    std::shared_ptr<LaneLine> getRightLane() const;

private:
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
    cv::Mat depth;
    cv::Mat rgb;
    cv::Mat debug;
    cv::Mat birdview;

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};
    
    int minLaneInShadow[3] = {90, 35, 95};
    int maxLaneInShadow[3] = {180, 117, 158};

    int lowThreshold = 2;
    int votes = 60;
    int minLinlength = 60;
    int maxLineGap = 5;
    size_t sumLaneWidth = 0;
    size_t frameCount = 0;

    int usebirdview = 1;

    int offsetX = 160;
    int offsetY = 180;
    int birdwidth = 300;
    int birdheight = 330;
    int skyline = 100;
};
#endif
