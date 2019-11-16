#ifndef DETECTLANE_H
#define DETECTLANE_H

#include "opencv2/ximgproc/segmentation.hpp"
#include "opencv2/core.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include <memory>
#include <iostream>

using namespace std;
using namespace cv;
using namespace cv::ximgproc::segmentation;

class LaneLine;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();
    
    Point calculateError();
    void processDepth();
    void updateDepth(const Mat& depth);
    void updateRGB(const Mat& rgb);
private:
    Mat preprocess(const Mat& src);
    Mat shadow(const Mat& src);
    Mat birdviewTransformation(const Mat& src);
    Mat morphological(const Mat& img);
    Mat ROI(const Mat& src);

    void drawLine(float slope, float yintercept, Mat& HoughTransform);
    Point Hough(const Mat& img, const Mat& src);
    int detectSigns(const Mat& src);


    LaneLine* leftLane;
    LaneLine* rightLane;

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
