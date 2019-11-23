#ifndef DETECT_SIGN_H
#define DETECT_SIGN_H

#include <opencv2/core.hpp>

class DetectSign
{
public:
    void updateRGB(const cv::Mat& rgb);
    int detect();

private:
    cv::Mat rgb;
    int minBlue[3] = {100, 90, 35};
    int maxBlue[3] = {240, 255, 255};
};



#endif