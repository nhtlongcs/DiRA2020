#include "planning/planning.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "cds_msgs/ResetLane.h"
#include "cds_msgs/GetDrivePoint.h"
#include "cds_msgs/RecoverLane.h"
#include "car_control/GetDriveSpeed.h"
#include "lane_detect/lane_detect.h"
#include "common/libcommon.h"
using namespace std;

constexpr const char *CONF_PLAN_WINDOW = "ConfigPlanning";

Planning::Planning()
    : _nh{"planning"}
    , _configServer{_nh}
    , isAvoidObjectDone{true}, isTurningDone{true}, prevObject{0}, object{0}, laneToDriveCloseTo{2}
{
    _resetLaneClient = _nh.serviceClient<cds_msgs::ResetLane>("reset_lane");
    _recoverClient = _nh.serviceClient<cds_msgs::GetDrivePoint>("recover");

    _configServer.setCallback(std::bind(&Planning::configCallback, this, std::placeholders::_1, std::placeholders::_2));
    // cv::namedWindow(CONF_PLAN_WINDOW);
    // cv::createTrackbar("AvoidTime", CONF_PLAN_WINDOW, &avoidObjectTime, 50);
    // cv::createTrackbar("TurningTime", CONF_PLAN_WINDOW, &turningTime, 100);
    // cv::createTrackbar("LaneToCloseTo", CONF_PLAN_WINDOW, &laneToDriveCloseTo, 2);

    _objectTimer = _nh.createTimer(ros::Duration{avoidObjectTime / 10.0f}, &Planning::onObjectTimeout, this, true);
    _turnTimer = _nh.createTimer(ros::Duration{turningTime / 10.0f}, &Planning::onTurnTimeout, this, true);
}

void Planning::updateSign(int signId)
{
    prevSign = sign;
    sign = signId;
}

void Planning::planning()
{
    
    // laneDetect->detect(lastPriority);
    // auto laneWidth = laneDetect->getLaneWidth();
    // auto leftLane = laneDetect->getLeftLane();
    // auto rightLane = laneDetect->getRightLane();

    prevObject = object;
    // TODO: fix
    // object = objectDetect->detect();

    ROS_INFO("object = %d", object);
    ROS_INFO("sign = %d", sign);

    // bool object = false;

    // if (object)
    // {
    //     if (isAvoidObjectDone)
    //     {
    //         prevObject = object;
    //         isAvoidObjectDone = false;
    //         _objectTimer.stop();
    //         _objectTimer.setPeriod(ros::Duration{avoidObjectTime/10.0f});
    //         _objectTimer.start();
    //     }
    // }

    // if (!isAvoidObjectDone)
    // {
    //     object = prevObject;
    // }

    int notuse;
    bool isTurnable = laneDetect->isAbleToTurn(this->depth);
    ROS_INFO_ONCE("Is turnable = %d", isTurnable);
    // kento fix this
    short_term_memory.push_back(sign);
    if (short_term_memory.size() > KENTODEEPTRY)
    {
        short_term_memory.pop_front();
    }

    if (sign != 0)
    {
        if (isTurningDone)
        {
            prevSign = sign; // important
            isTurningDone = false;

            requestResetLane(sign);
            // leftLane->reset();
            // rightLane->reset();
            // laneDetect->detect(sign);
            driveSpeed = minSpeed;

            _turnTimer.stop();
            _turnTimer.setPeriod(ros::Duration{turningTime / 10.0f});
            _turnTimer.start();
        }
    }

    if (sign != 0)
    {
        ROS_INFO_ONCE("Slow down..............");
        driveSpeed = minSpeed;
    }

    // if (prevSign != 0 && sign == 0)
    // {
    //     if (isTurningDone)
    //     {
    //         turnSign = prevSign;
    //         ROS_INFO("I will turn %s", turnSign > 0 ? "right" : "left");
    //         isTurningDone = false;
    //         driveSpeed = minSpeed;
    //         // TODO: set timeout in case not found any able to turn
    //     }
    // }

    // if (!isTurningDone)
    // {
    //     if (laneDetect->isAbleToTurn(sign) && !isTurning)
    //     {
    //         ROS_INFO("It\'s time to turn");
    //         isTurning = true;
    //         _turnTimer.stop();
    //         _turnTimer.setPeriod(ros::Duration{turningTime/10.0f});
    //         _turnTimer.start();
    //     }

    if (isTurning)
    {
        driveSpeed = minSpeed;
        sign = prevSign;
        // drivePoint = cv::Point{330 * prevSign, 200};
        // return;
    }
    // }

    if (!isTurningDone)
    {
        sign = prevSign;
        // leftLane->reset();
        // rightLane->reset();
        if (sign > 0)
            requestResetLane(1);
            // rightLane->reset();
        else if (sign < 0)
            // leftLane->reset();
            requestResetLane(-1);
        laneDetect->detect(sign);
        driveSpeed = minSpeed;
    }
    else
    {
        driveSpeed = maxSpeed;
    }
    if (sign == 0)
    {

        int cntLeft = std::count(short_term_memory.begin(), short_term_memory.end(), -1);
        int cntRight = std::count(short_term_memory.begin(), short_term_memory.end(), 1);
        int cntStraight = std::count(short_term_memory.begin(), short_term_memory.end(), 0);
        if (isTurnable && !isTurningDone && (cntLeft != 0 || cntRight != 0))
        {

            // int max = std::max(cntLeft, std::max(cntRight, cntStraight));

            if (cntRight > 0)
            {
                driveSpeed = minSpeed;
                drivePoint = turnRight();
            }
            else if (cntLeft > 0)
            {
                driveSpeed = minSpeed;
                drivePoint = turnRight();
            }
            // if (max == 1)
            // {
            //     driveSpeed = minSpeed;
            //     drivePoint = turnRight();
            // }
            // else if (max == -1)
            // {
            //     driveSpeed = minSpeed;
            //     drivePoint = turnLeft();
            // }
        }
        else
        {
            driveSpeed = maxSpeed;
            if (leftParams && rightParams)
            {
                drivePoint = driveStraight(object);
            }
            else if (leftParams)
            {
                if (rightLane->recover(leftLane, laneWidth))
                {
                    drivePoint = driveStraight(object);
                }
                else
                {
                    drivePoint = driveCloseToLeft();
                }
            }
            else if (rightParams)
            {
                if (leftLane->recover(rightLane, laneWidth))
                {
                    drivePoint = driveStraight(object);
                }
                else
                {
                    drivePoint = driveCloseToRight();
                }
            }
            else
            {
                ROS_INFO("BOTH LANES NOT FOUND!");
                // drivePoint = cv::Point{330 * sign, 200};
            }
        }
    }
    else if (sign > 0)
    {
        driveSpeed = minSpeed;

        drivePoint = driveCloseToRight();
        // drivePoint = turnRight();
    }
    else
    {
        driveSpeed = minSpeed;
        // drivePoint = turnLeft();
        drivePoint = driveCloseToLeft();
    }
}

cv::Point Planning::driveCloseToLeft()
{
    ROS_INFO("DRIVE CLOSE TO THE LEFT SIDE");
    cv::Point leftDrive{0, 120};
    if (requestDrivePoint(-1, leftDrive))
    {
        leftDrive = cv::Point{leftDrive.x + 30, leftDrive.y};
    }

    return leftDrive;
}

cv::Point Planning::driveCloseToRight()
{
    ROS_INFO("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point rightDrive{319, 120};
    if (requestDrivePoint(1, rightDrive))
    {
        rightDrive = cv::Point{rightDrive.x - 30, rightDrive.y};
    }

    return rightDrive;
}

cv::Point Planning::driveStraight(int object)
{
    ROS_INFO_COND(object != 0, "DRIVE STRAIGHT %d", object);
    if (object > 0)
    {
        return driveCloseToLeft();
    }
    else if (object < 0)
    {
        return driveCloseToRight();
    }

    if (laneToDriveCloseTo == 0)
    {
        return driveCloseToLeft();
    }
    else if (laneToDriveCloseTo == 1)
    {
        // cv::Point leftDrive, rightDrive;
        // return driveCloseToRight();
        LineParams midParams;
        for (int i = 0; i < midParams.size(); i++)
            midParams[i] = (*leftParams)[i] + (*rightParams)[i];
        int x = getXByY(midParams, 70); // TODO: change to ros::getParam
        return cv::Point{x, 70};
    }
    else
    {
        return driveCloseToRight();
    }
}

cv::Point Planning::turnLeft()
{
    ROS_INFO_ONCE("TURN LEFT");
    requestResetLane(-1);
    // laneDetect->getRightLane()->reset();
    // laneDetect->getLeftLane()->reset();
    // laneDetect->detect(-1);
    if (leftParams)
    {
        return driveCloseToLeft();
    }
    else
    {
        return cv::Point{0, 120};
    }
}

cv::Point Planning::turnRight()
{
    ROS_INFO_ONCE("TURN RIGHT");
    requestResetLane(1);
    // laneDetect->getRightLane()->reset();
    // laneDetect->getLeftLane()->reset();
    // laneDetect->detect(1);

    if (rightParams)
    {
        cv::Point rightDrive;
        if (requestDrivePoint(1, rightDrive))
        {
            rightDrive = cv::Point{rightDrive.x, rightDrive.y};
            return rightDrive;
        }
    }
    else
    {
        return cv::Point{319, 120};
    }
}

void Planning::onObjectTimeout(const ros::TimerEvent &event)
{
    ROS_INFO("Object timeout");
    isAvoidObjectDone = true;
}

void Planning::onTurnTimeout(const ros::TimerEvent &event)
{
    ROS_INFO("Turn timeout");
    isTurningDone = true;
    turnSign = 0;
    isTurning = false;
}

void Planning::configCallback(planning::PlanningConfig &config, uint32_t level)
{
    turningTime = config.turning_time;
    avoidObjectTime = config.avoid_time;
    laneToDriveCloseTo = config.lane_close;
}

void Planning::laneCallback(const cds_msgs::lane& msg)
{
    if (msg.left_params.empty())
    {
        leftParams = nullptr;
    } else
    {
        if (!leftParams)
        {
            leftParams = std::make_shared<LineParams>();
        }
        std::copy_n(msg.left_params.begin(), leftParams->size(), leftParams->begin());
    }
    
    if (msg.right_params.empty())
    {
        rightParams = nullptr;
    } 
    else
    {
        if (!rightParams)
        {
            rightParams = std::make_shared<LineParams>();
        }
        std::copy_n(msg.right_params.begin(), rightParams->size(), rightParams->begin());
    }
    
}

void Planning::signCallback(const cds_msgs::sign& msg)
{

}

void Planning::objectCallback(const cds_msgs::object& msg)
{

}

void Planning::requestResetLane(int lane)
{
    cds_msgs::ResetLane srv;
    srv.request.lane = lane;
    if (_resetLaneClient.call(srv))
    {
        leftParams = std::make_shared<LineParams>();
        if (lane < 0)
        {
            std::copy(srv.response.left_params.begin(), srv.response.left_params.end(), leftParams->begin());
        }
        else if (lane > 0)
        {
            std::copy(srv.response.right_params.begin(), srv.response.right_params.end(), rightParams->begin());
        }
        else
        {
            std::copy(srv.response.left_params.begin(), srv.response.left_params.end(), leftParams->begin());
            std::copy(srv.response.right_params.begin(), srv.response.right_params.end(), rightParams->begin());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service reset_lane");
    }
}

bool Planning::requestRecover(int lane)
{
    cds_msgs::RecoverLane srv;
    srv.request.lane = lane;
    if (_recoverClient.call(srv))
    {
        if (srv.response.lane == -1)
        {
            std::copy(srv.response.params.begin(), srv.response.params.end(), leftParams->begin());
        } else if (srv.response.lane == 1)
        {
            std::copy(srv.response.params.begin(), srv.response.params.end(), rightParams->begin());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service drive_point");
    }
}

bool Planning::requestDrivePoint(int lane, cv::Point& resultPoint)
{
    cds_msgs::GetDrivePoint srv;
    srv.request.lane = lane;
    if (_drivePointClient.call(srv))
    {
        if (srv.response.found)
        {
            resultPoint.x = srv.response.point.x;
            resultPoint.y = srv.response.point.y;
            return true;
        } else
        {
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service drive_point");
    }
    return false;
}

bool Planning::requestDriveSpeed(float& speed)
{
    cds_msgs::
}
