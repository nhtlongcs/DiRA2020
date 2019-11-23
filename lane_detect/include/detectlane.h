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
    void show(cv::Mat& colorBirdview) const;
    void updateDepth(const cv::Mat& depth);
    void updateRGB(const cv::Mat& rgb);

    // std::shared_ptr<LaneLine> getLeftLane();
    // std::shared_ptr<LaneLine> getRightLane();
    cv::Point calculateError(cv::Point carPos);
    
private:
    void processDepth();

    cv::Mat preprocess(const cv::Mat& src);
    cv::Mat shadow(const cv::Mat& src);
    cv::Mat birdviewTransformation(const cv::Mat& src);
    cv::Mat morphological(const cv::Mat& img);
    cv::Mat ROI(const cv::Mat& src);

    void drawLine(float slope, float yintercept, cv::Mat& HoughTransform);
    cv::Point Hough(const cv::Mat& img, const cv::Mat& src);


    std::shared_ptr<LaneLine> leftLane;
    std::shared_ptr<LaneLine> rightLane;

    cv::Mat depth;
    cv::Mat rgb;
    cv::Mat debug;

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};
    
    int minLaneInShadow[3] = {90, 35, 95};
    int maxLaneInShadow[3] = {180, 117, 158};

    int lowThreshold = 2;
    int votes = 60;
    int minLinlength = 60;
    int maxLineGap = 5;
    int laneWidth = 0;
    size_t frameCount = 0;

    const int offsetX = 160;
    const int offsetY = 180;
    const int birdwidth = 300;
    const int birdheight = 330;
    const int skyline = 95;
};
#endif
