#include "planning.h"
#include "detectobject.h"
#include "detectlane.h"
#include "laneline.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
using namespace std;

constexpr const char *CONF_PLAN_WINDOW = "ConfigPlanning";

Planning::Planning(DetectLane *laneDetect, DetectObject *objectDetect)
    : laneDetect{laneDetect}, objectDetect{objectDetect}, sign{0}, prevSign{0}, isAvoidObjectDone{true}, isTurningDone{true}, prevObject{0}, object{0}, laneToDriveCloseTo{2}, _nh{"planning"}, _configServer{_nh}
{
    _configServer.setCallback(boost::bind(&Planning::configCallback, this, _1, _2));

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

void Planning::planning(cv::Point &drivePoint, int &driveSpeed, int maxSpeed, int minSpeed)
{
    laneDetect->detect();
    auto laneWidth = laneDetect->getLaneWidth();
    auto leftLane = laneDetect->getLeftLane();
    auto rightLane = laneDetect->getRightLane();

    prevObject = object;
    object = objectDetect->detect();

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

            leftLane->reset();
            rightLane->reset();
            laneDetect->detect();
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
        leftLane->reset();
        rightLane->reset();
        laneDetect->detect();
        driveSpeed = minSpeed;
    }
    else
    {
        driveSpeed = maxSpeed;
    }
    if (sign == 0)
    {

        if (isTurnable)
        {

            int cntLeft = std::count(short_term_memory.begin(), short_term_memory.end(), -1);
            int cntRight = std::count(short_term_memory.begin(), short_term_memory.end(), 1);
            int cntStraight = std::count(short_term_memory.begin(), short_term_memory.end(), 0);

            int max = std::max(cntLeft, std::max(cntRight, cntStraight));
            
            if (max == 1)
            {
                driveSpeed = minSpeed;
                drivePoint = turnRight();
            }
            else if (max == -1)
            {
                driveSpeed = minSpeed;
                drivePoint = turnLeft();
            }
            
        } else
        {
            driveSpeed = maxSpeed;
            if (leftLane->isFound() && rightLane->isFound())
            {
                drivePoint = driveStraight(object);
            }
            else if (leftLane->isFound())
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
            else if (rightLane->isFound())
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

void Planning::updateBinary(cv::Mat binaryImage)
{
    // cerr << "Check\n";
    this->laneDetect->updateBinary(binaryImage);
}

void Planning::updateColor(cv::Mat colorImage)
{
    this->laneDetect->updateRGB(colorImage);
    // this->signDetect->updateRGB(colorImage);
}

void Planning::updateDepth(cv::Mat depthImage)
{
    this->depth = depthImage.clone();
    // this->objectDetect->update(depthImage);
    // this->signDetect->updateDepth(depthImage);
}

cv::Point Planning::driveCloseToLeft()
{
    ROS_INFO("DRIVE CLOSE TO THE LEFT SIDE");
    cv::Point leftDrive{0, 120};
    if (laneDetect->getLeftLane()->getDrivePoint(leftDrive))
    {
        leftDrive = cv::Point{leftDrive.x + 30, leftDrive.y};
    }

    return leftDrive;
}

cv::Point Planning::driveCloseToRight()
{
    ROS_INFO("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point rightDrive{319, 120};
    if (laneDetect->getRightLane()->getDrivePoint(rightDrive))
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
        cv::Point leftDrive, rightDrive;
        // return driveCloseToRight();
        laneDetect->getLeftLane()->getDrivePoint(leftDrive);
        laneDetect->getRightLane()->getDrivePoint(rightDrive);
        return (leftDrive + rightDrive) / 2;
    }
    else
    {
        return driveCloseToRight();
    }
}

cv::Point Planning::turnLeft()
{
    ROS_INFO_ONCE("TURN LEFT");
    laneDetect->getRightLane()->reset();
    laneDetect->getLeftLane()->reset();
    laneDetect->detect();
    if (laneDetect->getLeftLane()->isFound())
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
    laneDetect->getRightLane()->reset();
    laneDetect->getLeftLane()->reset();
    laneDetect->detect();

    if (laneDetect->getRightLane()->isFound())
    {
        cv::Point rightDrive;
        if (laneDetect->getRightLane()->getDrivePoint(rightDrive))
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

void Planning::configCallback(lane_detect::planningConfig &config, uint32_t level)
{
    turningTime = config.turning_time;
    avoidObjectTime = config.avoid_time;
    laneToDriveCloseTo = config.lane_close;
}
