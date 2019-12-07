#ifndef DETECT_SIGN_H
#define DETECT_SIGN_H

#include <opencv2/core.hpp>
#include <list>

class DetectSign
{
public:
    DetectSign(const std::string& leftPath, const std::string& rightPath);
    void updateRGB(const cv::Mat& rgb);
    int detect();

private:
    int detectOneFrame();
    

private:
    cv::Mat rgb;

    cv::Mat LEFT_TEMPLATE, RIGHT_TEMPLATE;
    
    int MAX_DIFF = 0;
    const int MAX_FRAME_COUNT = 7;
    std::list<int> recentDetects;


    int frameCount = 0;
    int minBlue[3] = {100, 90, 35};
    int maxBlue[3] = {240, 255, 255};
};



#endif