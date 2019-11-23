#ifndef DETECTLANE_H
#define DETECTLANE_H

#include "opencv2/core.hpp"

class LaneLine;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();
    
    cv::Point calculateError();
    void processDepth();
    void updateDepth(const cv::Mat& depth);
    void updateRGB(const cv::Mat& rgb);
private:
    cv::Mat preprocess(const cv::Mat& src);
    cv::Mat shadow(const cv::Mat& src);
    cv::Mat birdviewTransformation(const cv::Mat& src);
    cv::Mat morphological(const cv::Mat& img);
    cv::Mat ROI(const cv::Mat& src);

    void drawLine(float slope, float yintercept, cv::Mat& HoughTransform);
    cv::Point Hough(const cv::Mat& img, const cv::Mat& src);
    int detectSigns(const cv::Mat& src);


    LaneLine* leftLane;
    LaneLine* rightLane;
    LaneLine* midLane;

    cv::Mat depth;
    cv::Mat rgb;
    cv::Mat debug;

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};
    int minBlue[3] = {100, 90, 35};
    int maxBlue[3] = {240, 255, 255};
    int minLaneInShadow[3] = {90, 35, 95};
    int maxLaneInShadow[3] = {180, 117, 158};

    int lowThreshold = 2;
    int votes = 60;
    int minLinlength = 60;
    int maxLineGap = 5;

    const int offsetX = 160;
    const int offsetY = 180;
    const int birdwidth = 300;
    const int birdheight = 330;
    const int skyline = 95;
};
#endif
