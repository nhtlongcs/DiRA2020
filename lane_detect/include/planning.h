#ifndef PLANNING_H
#define PLANNING_H

#include <opencv2/core.hpp>
#include <ros/ros.h>

class DetectLane;
class DetectObject;

class Planning
{
public:
    Planning(DetectLane *laneDetect, DetectObject *objectDetect);

    void updateBinary(cv::Mat binaryImage);
    void updateColor(cv::Mat colorImage);
    void updateDepth(cv::Mat depthImage);
    void updateSign(int signId);
    void planning(cv::Point &drivePoint, int &speed, int maxSpeed, int minSpeed);

private:
    void onTurnTimeout(const ros::TimerEvent& event);
    void onObjectTimeout(const ros::TimerEvent& event);

private:
    cv::Point driveCloseToLeft();
    cv::Point driveCloseToRight();
    cv::Point driveStraight(int object);
    cv::Point turnLeft();
    cv::Point turnRight();

private:

    ros::NodeHandle _nh;
    ros::Timer _objectTimer, _turnTimer;

    DetectLane *laneDetect;
    DetectObject *objectDetect;

    bool isAvoidObjectDone, isTurningDone;
    int turningTime = 40, avoidObjectTime = 35; // 1/10 seconds

    int countTurning, delay; // for turning
    int prevSign, sign;      // for signDetect
    int prevObject, object;      // for signDetect
    int rate;

    int turnSign = 0;
    bool isTurning = false;
};

#endif