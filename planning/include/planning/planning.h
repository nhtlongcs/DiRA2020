#ifndef PLANNING_H
#define PLANNING_H

#include <opencv2/core.hpp>
#include <cds_msgs/lane.h>
#include <cds_msgs/object.h>
#include <cds_msgs/sign.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <planning/PlanningConfig.h>
#include <list>
#define KENTODEEPTRY 5

class LineParams;

class Planning
{
public:
    Planning();

    void updateBinary(cv::Mat binaryImage);
    void updateColor(cv::Mat colorImage);
    void updateDepth(cv::Mat depthImage);
    void updateSign(int signId);
    void planning(cv::Point &drivePoint, int &speed, int maxSpeed, int minSpeed);

public:
    void configCallback(planning::PlanningConfig& config, uint32_t level);
    void laneCallback(const cds_msgs::lane& msg);

private:
    void requestResetLane(int lane) const;

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
    dynamic_reconfigure::Server<planning::PlanningConfig> _configServer;
    ros::Timer _objectTimer, _turnTimer;

    std::unique_ptr<LineParams> leftParams, rightParams;
    std::vector<cv::Rect> objectBoxes;

    bool isAvoidObjectDone, isTurningDone;
    int turningTime = 40, avoidObjectTime = 35; // 1/10 seconds

    int countTurning, delay; // for turning
    int prevSign, sign;      // for signDetect
    int prevObject, object;      // for signDetect
    int rate;

    int laneToDriveCloseTo;

    int turnSign = 0;
    std::list<int> short_term_memory;
    bool isTurning = false;

    cv::Mat depth;

    int lastPriority = 0;
};

#endif