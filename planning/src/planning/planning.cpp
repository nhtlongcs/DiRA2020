#include "planning/planning.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include "cds_msgs/ResetLane.h"
#include "cds_msgs/RecoverLane.h"
#include "cds_msgs/IsTurnable.h"
#include "common/libcommon.h"
using namespace std;

static cv::Mat debugImage;
constexpr const char *CONF_PLAN_WINDOW = "ConfigPlanning";

Planning::Planning()
    : _nh{"planning"}, _configServer{_nh}, prevObject{0}, object{0}, laneToDriveCloseTo{RIGHT}
    , objectState{AvoidObjectState::DONE}
    , turningState{TurningState::DONE}
{
    _resetLaneClient = _nh.serviceClient<cds_msgs::ResetLane>("reset_lane");
    _controlPub = _nh.advertise<geometry_msgs::Twist>("/control", 1);
    _laneSub = _nh.subscribe("/lane_detect/lane", 1, &Planning::laneCallback, this);
    _signSub = _nh.subscribe("/sign_detect/sign", 1, &Planning::signCallback, this);
    _objSub = _nh.subscribe("/object_detect/object", 1, &Planning::objectCallback, this);

    _configServer.setCallback(std::bind(&Planning::configCallback, this, std::placeholders::_1, std::placeholders::_2));
    _objectTimer = _nh.createTimer(ros::Duration{avoidObjectTime / 10.0f}, &Planning::onObjectTimeout, this, true);
    _turnTimer = _nh.createTimer(ros::Duration{turningTime / 10.0f}, &Planning::onTurnTimeout, this, true);
}

void Planning::planning()
{
    float min_speed = 0.0f, max_speed = 30.0f;
    if (!ros::param::getCached("/car_control/min_velocity", min_speed))
    {
        ROS_WARN("Cannot get min_velocity from ParameterServer, use default = 0.0f");
    }

    if (!ros::param::getCached("/car_control/max_velocity", max_speed))
    {
        ROS_WARN("Cannot get max_velocity from ParameterServer, use default = 30.0f");
    }

    if (turningState != TurningState::DONE)
    {
        planningWhileTurning(min_speed, max_speed);
    } else
    {
        planningWhileDriveStraight(min_speed, max_speed);
    }
}

void Planning::planningWhileDriveStraight(const float& min_speed, const float& max_speed)
{
    if (!leftParams || !rightParams)
    {
        ROS_WARN("Both lanes not found. Not planning");
        return;
    }

    cv::Point drivePoint;

    if (leftParams && rightParams)
    {
        ROS_DEBUG("Both lanes found. DriveStraight");
        drivePoint = driveStraight();
    }
    else if (leftParams)
    {
        ROS_DEBUG("Only left found.");
        if (!requestRecover(RIGHT))
        {
            drivePoint = driveStraight();
        }
        else
        {
            drivePoint = driveCloseToLeft();
        }
    }
    else if (rightParams)
    {
        ROS_DEBUG("Only right found.");
        if (!requestRecover(LEFT))
        {
            drivePoint = driveStraight();
        }
        else
        {
            drivePoint = driveCloseToRight();
        }
    }

    publishMessage(drivePoint, max_speed);
}

void Planning::planningWhileTurning(const float& min_speed, const float& max_speed)
{
    cv::Point drivePoint;
    switch (turningState)
    {
        case TurningState::NOT_TURNING:
        {
            ROS_DEBUG("TurningState: NOT_TURNING");
            bool isTurnable = requestIsAbleToTurn(sign);
            ROS_DEBUG("Is turnable = %d", isTurnable);

            if (isTurnable)
            {
                _turnTimer.stop();
                _turnTimer.setPeriod(ros::Duration{turningTime / 10.0f});
                _turnTimer.start();

                turningState = TurningState::IS_TURNING;
                turningDirect = sign;
            }
            break;
        }
        case TurningState::IS_TURNING:
        {
            ROS_DEBUG("TurningState: IS_TURNING");
            if (turningDirect > 0)
            {
                drivePoint = turnRight();
            } else
            {
                drivePoint = turnLeft();
            }
            break;
        }
        case TurningState::DONE:
        {
            ROS_DEBUG("TurningState: DONE");
            break;
        }
        default:
        {
            ROS_ERROR("Unknow turning state: %d", static_cast<int>(turningState));
            break;
        }
    }
    
    publishMessage(drivePoint, min_speed);
}

cv::Point Planning::driveCloseToLeft()
{
    ROS_DEBUG("DRIVE CLOSE TO THE LEFT SIDE");
    cv::Point leftDrive{0, drivePointY};
    if (leftParams != nullptr)
    {
        leftDrive.x = getXByY(*leftParams, drivePointY) + 30;
    }
    return leftDrive;
}

cv::Point Planning::driveCloseToRight()
{
    ROS_DEBUG("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point rightDrive{319, drivePointY};
    if (rightParams != nullptr)
    {
        rightDrive.x = getXByY(*rightParams, drivePointY) - 30;
    }
    return rightDrive;
}

cv::Point Planning::driveStraight()
{
    ROS_DEBUG_COND(object != 0, "DRIVE STRAIGHT OBJECT = %d", object);
    switch (objectState)
    {
        case AvoidObjectState::READY:
        {
            ROS_DEBUG("ObjectState: READY");

            _objectTimer.stop();
            _objectTimer.setPeriod(ros::Duration{avoidObjectTime / 10.0f});
            _objectTimer.start();
            
            objectState = AvoidObjectState::ON_AVOIDING;
            objectDirect = object;

            break;
        }
        case AvoidObjectState::ON_AVOIDING:
        {
            ROS_DEBUG("On avoiding");
            if (objectDirect == RIGHT)
            {
                return driveCloseToLeft();
            }
            else if (objectDirect == LEFT)
            {
                return driveCloseToRight();
            }
            break;
        }
        case AvoidObjectState::DONE:
        {
            ROS_DEBUG("DriveStraight without object");
            break;
        }
        default:
        {
            ROS_ERROR("Unknow object state = %d", static_cast<int>(objectState));
            break;
        }
    }

    if (laneToDriveCloseTo == LEFT)
    {
        return driveCloseToLeft();
    } else if (laneToDriveCloseTo == RIGHT)
    {
        return driveCloseToRight();
    } else
    {
        LineParams midParams;
        for (int i = 0; i < midParams.size(); i++)
            midParams[i] = ((*leftParams)[i] + (*rightParams)[i]) / 2.0f;
        int x = getXByY(midParams, drivePointY); // TODO: change to ros::getParam
        return cv::Point{x, drivePointY};
    }
}

cv::Point Planning::turnLeft()
{
    ROS_DEBUG("TURN LEFT");
    if (leftParams)
    {
        // return cv::Point{0, drivePointY};
        return driveCloseToLeft();
    }
    else
    {
        return cv::Point{0, drivePointY};
    }
}

cv::Point Planning::turnRight()
{
    ROS_DEBUG("TURN RIGHT");
    if (rightParams)
    {
        // cv::Point rightDrive{319, drivePointY};
        return driveCloseToRight();
    }
    else
    {
        return cv::Point{319, drivePointY};
    }
}

void Planning::onObjectTimeout(const ros::TimerEvent &event)
{
    ROS_DEBUG("Object timeout");
    objectState = AvoidObjectState::DONE;
}

void Planning::onTurnTimeout(const ros::TimerEvent &event)
{
    ROS_DEBUG("Turn timeout");
    turningState = TurningState::DONE;
}

void Planning::configCallback(planning::PlanningConfig &config, uint32_t level)
{
    turningTime = config.turning_time;
    avoidObjectTime = config.avoid_time;
    laneToDriveCloseTo = config.lane_close - 1; // config unable to set negative value
}

void Planning::laneCallback(const cds_msgs::lane &msg)
{
    if (msg.left_params.empty())
    {
        leftParams = nullptr;
    }
    else
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

void Planning::signCallback(const cds_msgs::sign &msg)
{
    prevSign = sign;
    sign = msg.sign_id;

    if (sign != 0)
    {
        ROS_DEBUG("Receive Sign = %d", sign);
        turningState = TurningState::NOT_TURNING;
    }
}

void Planning::objectCallback(const std_msgs::Int8 &msg)
{
    prevObject = object;
    object = msg.data;

    if (object != 0)
    {
        ROS_DEBUG("Receive Object = %d", object);
        objectState = AvoidObjectState::READY;
    }
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
    // cds_msgs::RecoverLane srv;
    // srv.request.lane = lane;
    // if (_recoverClient.call(srv))
    // {
    //     if (srv.response.lane == -1)
    //     {
    //         std::copy(srv.response.params.begin(), srv.response.params.end(), leftParams->begin());
    //     }
    //     else if (srv.response.lane == 1)
    //     {
    //         std::copy(srv.response.params.begin(), srv.response.params.end(), rightParams->begin());
    //     }
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service drive_point");
    // }
    return false;
}

bool Planning::requestIsAbleToTurn(int direction)
{
    ros::ServiceClient client = _nh.serviceClient<cds_msgs::IsTurnable>("/lane_detect/IsTurnable");

    cds_msgs::IsTurnable srv;
    srv.request.direct = direction;
    if (!client.call(srv))
    {
        ROS_ERROR("Could not call IsTurnable Service");
        return false;
    }

    return srv.response.ok;
}

void Planning::publishMessage(const cv::Point& drivePoint, float speed)
{
    cv::Point carPos{160, 319};
    if (!_nh.getParamCached("/car_control/carpos_x", carPos.x))
    {
        ROS_WARN("Cannot get carpos_x, use default = 160");
    }

    if (!_nh.getParamCached("/car_control/carpos_y", carPos.y))
    {
        ROS_WARN("Cannot get carpos_y, use default = 319");
    }


    debugImage = cv::Mat::zeros(240, 320, CV_8UC3);
    cv::circle(debugImage, drivePoint, 20, cv::Scalar{0, 255, 255}, -1);
    cv::circle(debugImage, carPos, 20, cv::Scalar{0, 0, 255}, -1);
    cv::imshow("Driving", debugImage);
    cv::waitKey(1);
    
    float angle = errorAngle(carPos, drivePoint);
    ROS_DEBUG("Planning speed = %.2f, steer = %.2f", speed, angle);

    geometry_msgs::Twist msg;
    msg.linear.x = speed;
    msg.angular.z = angle * M_PI / 180.0f;

    _controlPub.publish(msg);
}