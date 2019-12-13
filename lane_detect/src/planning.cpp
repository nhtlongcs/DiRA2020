#include "planning.h"
#include "detectobject.h"
#include "detectlane.h"
#include "detectsign.h"
#include "laneline.h"

#include <ros/ros.h>

Planning::Planning(DetectLane* laneDetect, DetectObject* objectDetect, DetectSign* signDetect, int rate)
: laneDetect{laneDetect}
, objectDetect{objectDetect}
, signDetect{signDetect}
, sign{0}
, prevSign{0}
, countTurning{rate*3}
, rate{rate}
{
}

void Planning::planning(cv::Point& drivePoint, int& driveSpeed, int maxSpeed, int minSpeed)
{
    laneDetect->detect();
    auto laneWidth = laneDetect->getLaneWidth();
    auto leftLane = laneDetect->getLeftLane();
    auto rightLane = laneDetect->getRightLane();

    prevSign = sign;
    sign = signDetect->detect();
    bool object = objectDetect->detect();
    
    if (object)
    {
        ROS_INFO("object!!!");
    }

    if (sign != 0)
    {
        // NOTE: test only
        sign = 1;
    }

    if (sign != 0)
    {
        ROS_INFO("Turn %d", sign);
        if (sign != prevSign)
        {
            countTurning = rate * 3;
            delay = rate;
        }
        driveSpeed = minSpeed;
        leftLane->reset();
        rightLane->reset();
        laneDetect->detect();
    }
    else if (countTurning > 0)
    {
        sign = prevSign;
        if (delay > 0)
        {
            ROS_INFO("DELAY...");
            delay--;
        } else
        {
            ROS_INFO("TURNING...");
            countTurning--;
        }
    }
    

    if (leftLane->isFound() && rightLane->isFound())
    {
        if (sign == 0)
        {
            driveSpeed = maxSpeed;
            drivePoint = driveStraight(object);
        } else if (sign < 0)
        {
            drivePoint = turnLeft();
        } else
        {
            drivePoint = turnRight();
        }
    } else if (leftLane->isFound())
    {
        if (rightLane->recover(leftLane, laneWidth))
        {
            if (sign <= 0) // go straight or turn left
            {
                if (sign == 0)
                {
                    driveSpeed = maxSpeed;
                }
                drivePoint = driveStraight(object);
            } else
            {
                driveSpeed = minSpeed;
                drivePoint = turnRight();
            }
        } else
        {
            if (sign > 0)
            {
                ROS_INFO("TURN RIGHT BUT RIGHT LANE NOT FOUND!!");
            }
            drivePoint = driveCloseToLeft();
        }
    } else if (rightLane->isFound())
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
            } else
            {
                drivePoint = turnLeft();
            }
        } else
        {
            if (sign < 0)
            {
                ROS_INFO("TURN LEFT BUT LEFT LANE NOT FOUND!!");
            }
            drivePoint = driveCloseToRight();
        }
    } else
    {
        // ROS_INFO("BOTH LANES NOT FOUND!");
    }
}

void Planning::updateColor(cv::Mat colorImage)
{
    this->laneDetect->updateRGB(colorImage);
    this->signDetect->updateRGB(colorImage);
}

void Planning::updateDepth(cv::Mat depthImage)
{
    this->objectDetect->update(depthImage);
    this->signDetect->updateDepth(depthImage);
}

cv::Point Planning::driveCloseToLeft()
{
    // ROS_INFO("DRIVE CLOSE TO THE LEFT SIDE");
    cv::Point leftDrive;
    laneDetect->getLeftLane()->getDrivePoint(leftDrive);
    return {leftDrive.x + 30, leftDrive.y};
}

cv::Point Planning::driveCloseToRight()
{
    // ROS_INFO("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point rightDrive;
    laneDetect->getRightLane()->getDrivePoint(rightDrive);
    return {rightDrive.x - 30, rightDrive.y};
}

cv::Point Planning::driveStraight(bool object)
{
    // ROS_INFO("DRIVE STRAIGHT");
    // cv::Point leftDrive, rightDrive;
    // laneDetect->getLeftLane()->getDrivePoint(leftDrive);
    // laneDetect->getRightLane()->getDrivePoint(rightDrive);
    // return (leftDrive + rightDrive) / 2;
    if (object)
    {
        return driveCloseToLeft();
    }
    return driveCloseToRight();
}

cv::Point Planning::turnLeft()
{
    // ROS_INFO("TURN LEFT");
    if (laneDetect->getRightLane()->recover(laneDetect->getLeftLane(), laneDetect->getLaneWidth()))
    {
        return driveStraight(false);
    }
    return driveCloseToLeft();
}

cv::Point Planning::turnRight()
{
    // ROS_INFO("TURN RIGHT");
    if (laneDetect->getLeftLane()->recover(laneDetect->getRightLane(), laneDetect->getLaneWidth()))
    {
        return driveStraight(false);
    }
    return driveCloseToRight();
}