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
    void update(const cv::Mat& depth);
    int getDirect() const;
    bool detect();
    bool estimator(const cv::Mat& binaryROI);
private:
    bool detectOneFrame();

private:
    cv::Mat depth;
    cv::Rect objectROI;
    int kCluster;
    int configObjectmin[3];
    int configObjectmax[3];
    int detectThreshold;
    cv::Ptr<cv::BackgroundSubtractor> pBackSub;

    std::list<bool> objectHistories;
    int maxHistory = 10;
    int direct;

    DetectLane* lane;
};

#endif