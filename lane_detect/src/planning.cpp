#include "planning.h"
#include "detectobject.h"
#include "detectlane.h"
#include "laneline.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

constexpr const char* CONF_PLAN_WINDOW = "ConfigPlanning";

Planning::Planning(DetectLane *laneDetect, DetectObject *objectDetect)
    : laneDetect{laneDetect}, objectDetect{objectDetect}, sign{0}, prevSign{0}
    , isAvoidObjectDone{true}, isTurningDone{true}
    , prevObject{0}, object{0}
{
    cv::namedWindow(CONF_PLAN_WINDOW);
    cv::createTrackbar("AvoidTime", CONF_PLAN_WINDOW, &avoidObjectTime, 50);
    cv::createTrackbar("TurningTime", CONF_PLAN_WINDOW, &turningTime, 100);

    _objectTimer = _nh.createTimer(ros::Duration{avoidObjectTime/10.0f}, &Planning::onObjectTimeout, this, true);
    _turnTimer = _nh.createTimer(ros::Duration{turningTime/10.0f}, &Planning::onTurnTimeout, this, true);
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
            _turnTimer.setPeriod(ros::Duration{turningTime/10.0f});
            _turnTimer.start();
        }
    }

    // if (sign != 0)
    // {
    //     ROS_INFO("Slow down..............");
    //     driveSpeed = minSpeed;
    // }

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

    //     if (isTurning)
    //     {
    //         driveSpeed = 20;
    //         drivePoint = cv::Point{330 * prevSign, 200};
    //         return;
    //     }
    // }

    if (!isTurningDone)
    {
        sign = prevSign;
        leftLane->reset();
        rightLane->reset();
        laneDetect->detect();
        driveSpeed = minSpeed;
    } else
    {
        driveSpeed = maxSpeed;
    }

    if (leftLane->isFound() && rightLane->isFound())
    {
        if (sign == 0)
        {
            driveSpeed = maxSpeed;
            drivePoint = driveStraight(object);
        }
        else if (sign < 0)
        {
            // drivePoint = turnLeft();
            drivePoint = driveCloseToLeft();
        }
        else
        {
            // drivePoint = turnRight();
            drivePoint = driveCloseToRight();
        }
    }
    else if (leftLane->isFound())
    {
        // if (rightLane->recover(leftLane, laneWidth))
        // {
        //     if (sign <= 0) // go straight or turn left
        //     {
        //         if (sign == 0)
        //         {
        //             driveSpeed = maxSpeed;
        //         }
        //         drivePoint = driveStraight(object);
        //     }
        //     else
        //     {
        //         driveSpeed = minSpeed;
        //         drivePoint = turnRight();
        //     }
        // }
        // else
        // {
        //     if (sign > 0)
        //     {
        //         ROS_INFO("TURN RIGHT BUT RIGHT LANE NOT FOUND!!");
        //     }
        //     drivePoint = driveCloseToLeft();
        // }
        if (rightLane->recover(leftLane, laneWidth))
        {
            if (sign == 0)
            {
                driveSpeed = maxSpeed;
                drivePoint = driveStraight(object);
            } else if (sign < 0)
            {
                drivePoint = driveCloseToLeft();
            }
            else
            {
                driveSpeed = minSpeed;
                drivePoint = driveCloseToRight();
            }
        }
        else
        {
            if (sign > 0)
            {
                ROS_INFO("TURN RIGHT BUT RIGHT LANE NOT FOUND!!");
                drivePoint = cv::Point{330 * sign, 200};
            } else
            {
                drivePoint = driveCloseToLeft();
            }
        }
    }
    else if (rightLane->isFound())
    {
        if (leftLane->recover(rightLane, laneWidth))
        {
            if (sign >= 0) // go straight or turn right
            {
                if (sign == 0)
                {
                    driveSpeed = maxSpeed;
                }
                drivePoint = driveStraight(object);
            }
            else
            {
                drivePoint = turnLeft();
            }
        }
        else
        {
            if (sign < 0)
            {
                ROS_INFO("TURN LEFT BUT LEFT LANE NOT FOUND!!");
                drivePoint = cv::Point{330 * sign, 200};
            } else
            {
                drivePoint = driveCloseToRight();
            }
        }
    }
    else
    {
        // ROS_INFO("BOTH LANES NOT FOUND!");
        drivePoint = cv::Point{330 * sign, 200};
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
    // this->objectDetect->update(depthImage);
    // this->signDetect->updateDepth(depthImage);
}

cv::Point Planning::driveCloseToLeft()
{
    ROS_INFO("DRIVE CLOSE TO THE LEFT SIDE");
    cv::Point leftDrive;
    laneDetect->getLeftLane()->getDrivePoint(leftDrive);
    return {leftDrive.x + 30, leftDrive.y};
}

cv::Point Planning::driveCloseToRight()
{
    ROS_INFO("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point rightDrive;
    laneDetect->getRightLane()->getDrivePoint(rightDrive);
    return {rightDrive.x - 30, rightDrive.y};
}

cv::Point Planning::driveStraight(int object)
{
    ROS_INFO("DRIVE STRAIGHT %d", object);
    if (object > 0)
    {
        return driveCloseToLeft();
    } else if (object < 0)
    {
        return driveCloseToRight();
    }
    return driveCloseToRight();
    // cv::Point leftDrive, rightDrive;
    // laneDetect->getLeftLane()->getDrivePoint(leftDrive);
    // laneDetect->getRightLane()->getDrivePoint(rightDrive);
    // return (leftDrive + rightDrive) / 2;
}

cv::Point Planning::turnLeft()
{
    ROS_INFO("TURN LEFT");
    // if (laneDetect->getRightLane()->recover(laneDetect->getLeftLane(), laneDetect->getLaneWidth()))
    // {
    //     return driveStraight(false);
    // }
    return driveCloseToLeft();
}

cv::Point Planning::turnRight()
{
    ROS_INFO("TURN RIGHT");
    // if (laneDetect->getLeftLane()->recover(laneDetect->getRightLane(), laneDetect->getLaneWidth()))
    // {
    //     return driveStraight(false);
    // }
    return driveCloseToRight();
}

void Planning::onObjectTimeout(const ros::TimerEvent& event)
{
    ROS_INFO("Object timeout");
    isAvoidObjectDone = true;
}

void Planning::onTurnTimeout(const ros::TimerEvent& event)
{
    ROS_INFO("Turn timeout");
    isTurningDone = true;
    turnSign = 0;
    isTurning = false;
}