#ifndef PLANNING_H
#define PLANNING_H

#include <opencv2/core.hpp>
#include <cds_msgs/lane.h>
#include <cds_msgs/sign.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <planning/PlanningConfig.h>
#include <common/libcommon.h>
#include <list>
#include <memory>
#define KENTODEEPTRY 5

enum class TurningState : int{
    NOT_TURNING,
    IS_TURNING,
    DONE,
};

enum class AvoidObjectState : int{
    READY,
    ON_AVOIDING,
    DONE,
};

class Planning
{
public:
    Planning();
    void planning();
private:
    void planningWhileTurning(const float& min_speed, const float& max_speed);
    void planningWhileDriveStraight(const float& min_speed, const float& max_speed);

public:
    void configCallback(planning::PlanningConfig &config, uint32_t level);
    void laneCallback(const cds_msgs::lane &msg);
    void signCallback(const cds_msgs::sign &msg);
    void objectCallback(const std_msgs::Int8 &msg);

private:
    void requestResetLane(int lane);
    bool requestRecover(int lane);
    bool requestIsAbleToTurn(int direction);

private:
    void onTurnTimeout(const ros::TimerEvent &event);
    void onObjectTimeout(const ros::TimerEvent &event);

private:
    void publishMessage(const cv::Point& drivePoint, float speed);
    cv::Point driveCloseToLeft();
    cv::Point driveCloseToRight();
    cv::Point driveStraight();
    cv::Point turnLeft();
    cv::Point turnRight();

    cv::Point lastDrivePoint;

private:
    ros::NodeHandle _nh;
    ros::Publisher _controlPub;

    ros::Subscriber _laneSub;
    ros::Subscriber _signSub;
    ros::Subscriber _objSub;

    ros::ServiceClient _recoverClient, _resetLaneClient;
    dynamic_reconfigure::Server<planning::PlanningConfig> _configServer;
    ros::Timer _objectTimer, _turnTimer;

    std::shared_ptr<LineParams> leftParams, rightParams;
    std::vector<cv::Rect> objectBoxes;

    TurningState turningState;
    int turningDirect;
    int turningTime = 40; // 1/10 seconds
    
    AvoidObjectState objectState;
    int objectDirect;
    int avoidObjectTime = 35; // 1/10 seconds

    int drivePointY = 240 - 70;

    int countTurning, delay; // for turning
    int prevSign, sign;      // for signDetect
    int prevObject, object;  // for signDetect
    int rate;

    int laneToDriveCloseTo;

    int turnSign = 0;
    std::list<int> short_term_memory;

    int lastPriority = 0;
};

#endif