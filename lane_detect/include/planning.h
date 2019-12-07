#ifndef PLANNING_H
#define PLANNING_H

#include <opencv2/core.hpp>

class DetectLane;
class DetectSign;
class DetectObject;

class Planning
{
public:
    Planning(DetectLane* laneDetect, DetectObject* objectDetect, DetectSign* signDetect, int rate = 15);

    void updateColor(cv::Mat colorImage);
    void updateDepth(cv::Mat depthImage);

    void planning(cv::Point& drivePoint, int& speed, int maxSpeed, int minSpeed);

private:
    cv::Point driveCloseToLeft();
    cv::Point driveCloseToRight();
    cv::Point driveStraight();
    cv::Point turnLeft();
    cv::Point turnRight();

private:
    DetectLane* laneDetect;
    DetectObject* objectDetect;
    DetectSign* signDetect;

    int countTurning, delay; // for turning
    int prevSign, sign; // for signDetect
    int rate;
};


#endif