#ifndef DETECT_OBJECT_H
#define DETECT_OBJECT_H

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>

class DetectObject
{
public:
    DetectObject();
    void update(const cv::Mat& depth);
    bool detect();

private:
    cv::Mat depth;
    cv::Rect objectROI;
    int kCluster;
    int configObjectmin[3];
    int configObjectmax[3];
    int detectThreshold;
    cv::Ptr<cv::BackgroundSubtractor> pBackSub;
};

#endif