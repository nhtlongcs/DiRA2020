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
    , leftParams{nullptr}
    , rightParams{nullptr}
{
    std::string control_topic, crossroad_topic, sign_topic, lane_topic, object_topic;
    ROS_ASSERT(ros::param::get("/lane_detect_topic", lane_topic));
    ROS_ASSERT(ros::param::get("/signid_topic", sign_topic));
    ROS_ASSERT(ros::param::get("/object_topic", object_topic));
    ROS_ASSERT(ros::param::get("/control_topic", control_topic));
    ROS_ASSERT(ros::param::get("/crossroad", crossroad_topic));

    std::string recoverSrvTopic;
    ROS_ASSERT(ros::param::get("/recover_lane_srv", recoverSrvTopic));

    _resetLaneClient = _nh.serviceClient<cds_msgs::ResetLane>("reset_lane");
    _recoverClient = _nh.serviceClient<cds_msgs::RecoverLane>(recoverSrvTopic);

    _controlPub = _nh.advertise<geometry_msgs::Twist>(control_topic, 1);
    _laneSub = _nh.subscribe(lane_topic, 1, &Planning::laneCallback, this);
    _signSub = _nh.subscribe(sign_topic, 1, &Planning::signCallback, this);
    _objSub = _nh.subscribe(object_topic, 1, &Planning::objectCallback, this);
    _crossroadSub = _nh.subscribe(crossroad_topic, 1, &Planning::turnCallback, this);

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
    // planningWhileDriveStraight(min_speed, max_speed);

    if (turningState != TurningState::DONE)
    {
        planningWhileTurning(min_speed, max_speed);
    } 
    else
    {
        planningWhileDriveStraight(min_speed, max_speed);
    }
}

void Planning::planningWhileDriveStraight(const float& min_speed, const float& max_speed)
{
    if (!leftParams && !rightParams)
    {
        ROS_WARN("Both lanes not found. Not planning");
        publishMessage(lastDrivePoint, 0);
        return;
    }

    cv::Point drivePoint;

    if (leftParams || rightParams)
        drivePoint = driveStraight();
    // {
        // ROS_DEBUG("Both lanes found. DriveStraight");
    // }
    // else if (leftParams)
    // {
    //     ROS_DEBUG("Only left found.");
    //     drivePoint = driveCloseToLeft();
    // }
    // else if (rightParams)
    // {
    //     ROS_DEBUG("Only right found.");
    //     drivePoint = driveCloseToRight();
    // }

    ROS_INFO_STREAM("drivePoint: " << drivePoint);

    publishMessage(drivePoint, max_speed);
    lastDrivePoint = drivePoint;
}

void Planning::planningWhileTurning(const float& min_speed, const float& max_speed)
{
    cv::Point drivePoint;
    switch (turningState)
    {
        case TurningState::NOT_TURNING:
        {
            ROS_DEBUG("TurningState: NOT_TURNING");
            // bool isTurnable = requestIsAbleToTurn(sign);
            isTurnable = true;
            ROS_DEBUG("Is turnable = %d", isTurnable);

            // if (isTurnable)
            {
                _turnTimer.stop();
                _turnTimer.setPeriod(ros::Duration{turningTime / 10.0f});
                _turnTimer.start();

                turningState = TurningState::IS_TURNING;
                // turningDirect = sign;
            }
            // break;
        }
        case TurningState::IS_TURNING:
        {
            ROS_DEBUG("TurningState: IS_TURNING");
            if (isTurnable){
            if (sign == SignState::RIGHT)
            {
                drivePoint = turnRight();
            } 
            else if (sign == SignState::LEFT)
            {
                drivePoint = turnLeft();
            } 
            else if (sign == SignState::FORWARD || sign == SignState::NO_LEFT || sign == SignState::NO_RIGHT)
            {
                return;
            } 
            else if (sign == SignState::STOP)
            {
                drivePoint = cv::Point{160,10000};
                publishMessage(drivePoint, 0);
                return;
            }}
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
        leftDrive.x = getXByY(*leftParams, drivePointY) + 20;
    }
    return leftDrive;
}
cv::Point Planning::driveCloseToRightFromLeft()
{
    ROS_DEBUG("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point leftDrive{0, drivePointY};
    if (leftParams != nullptr)
    {
        leftDrive.x = getXByY(*leftParams, drivePointY) + 100;
    }
    return leftDrive;
}
cv::Point Planning::driveStraightFromLeft()
{
    // ROS_DEBUG("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point leftDrive{0, drivePointY};
    if (leftParams != nullptr)
    {
        leftDrive.x = getXByY(*leftParams, drivePointY) + 50;
    }
    return leftDrive;
}


cv::Point Planning::driveStraightFromRight()
{
    // ROS_DEBUG("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point rightDrive{319, drivePointY};
    if (rightParams != nullptr)
    {
        rightDrive.x = getXByY(*rightParams, drivePointY) - 50;
    }
    return rightDrive;
}
cv::Point Planning::driveCloseToRight()
{
    ROS_DEBUG("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point rightDrive{319, drivePointY};
    if (rightParams != nullptr)
    {
        rightDrive.x = getXByY(*rightParams, drivePointY) - 20;
    }
    return rightDrive;
}
cv::Point Planning::driveCloseToLeftFromRight()
{
    ROS_DEBUG("DRIVE CLOSE TO THE LEFT SIDE");
    cv::Point rightDrive{319, drivePointY};
    if (rightParams != nullptr)
    {
        rightDrive.x = getXByY(*rightParams, drivePointY) - 100;
    }
    return rightDrive;
}

cv::Point Planning::driveStraight()
{
    ROS_DEBUG_COND(object != 0, "DRIVE STRAIGHT OBJECT = %d", object);
    ROS_DEBUG_COND(object != 0, "OBJECT STATE = %d", objectState);
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

            // break;
        }
        case AvoidObjectState::ON_AVOIDING:
        {
            ROS_DEBUG("On avoiding");
            if (objectDirect == RIGHT)
            {
                if (leftParams)
                    return driveCloseToLeft();
                else
                    return driveCloseToLeftFromRight();
            }
            else if (objectDirect == LEFT)
            {
                if (rightParams)    
                    return driveCloseToRight();
                else 
                    return driveCloseToRightFromLeft();
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
        if (leftParams)
            return driveCloseToLeft();
        else
            return driveCloseToLeftFromRight();
    } else if (laneToDriveCloseTo == RIGHT)
    {
        if (rightParams)    
            return driveCloseToRight();
        else 
            return driveCloseToRightFromLeft();
    } else
    {
        if (rightParams && leftParams){ 
            LineParams midParams = (*leftParams + *rightParams) / 2.0;
            int x = getXByY(midParams, drivePointY);
            return cv::Point{x, drivePointY};
        }
        else if (rightParams){
            return driveStraightFromRight();
        }
        else if (leftParams){
            return driveStraightFromLeft();
        }
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
        return driveStraight();
        // return cv::Point{0, drivePointY};
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
        // return cv::Point{319, drivePointY};
        return driveStraight();

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
        for (int i = 0; i < msg.left_params.size(); i++)
        {
            (*leftParams)[i] = msg.left_params[i];
        }
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
        for (int i = 0; i < msg.right_params.size(); i++)
        {
            (*rightParams)[i] = msg.right_params[i];
        }
    }
}

void Planning::signCallback(const cds_msgs::sign &msg)
{
    prevSign = sign;
    sign = static_cast<SignState>(msg.sign_id);

    if (sign != SignState::NOSIGN)
    {
        ROS_DEBUG("Receive Sign = %d", sign);
        turningState = TurningState::NOT_TURNING;
    }
}

void Planning::turnCallback(const std_msgs::Int8 &msg)
{
    isTurnable = bool(msg.data);
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
    if (lane == LEFT)
    {
        srv.request.lane = srv.request.LEFT;
    } else if (lane == RIGHT)
    {
        srv.request.lane = srv.request.RIGHT;
    }
    if (_resetLaneClient.call(srv))
    {
        LineParams::ImplType params;
        if (srv.request.lane == srv.request.LEFT)
        {
            std::copy(srv.response.left_params.begin(), srv.response.left_params.end(), params.begin());
            leftParams = std::make_shared<LineParams>(params);
        }
        else if (srv.request.lane == srv.request.RIGHT)
        {
            std::copy(srv.response.right_params.begin(), srv.response.right_params.end(), params.begin());
            rightParams = std::make_shared<LineParams>(params);
        }
        else
        {
            std::copy(srv.response.left_params.begin(), srv.response.left_params.end(), params.begin());
            leftParams = std::make_shared<LineParams>(params);
            std::copy(srv.response.right_params.begin(), srv.response.right_params.end(), params.begin());
            rightParams = std::make_shared<LineParams>(params);
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
    if (lane == RIGHT)
    {
        ROS_DEBUG("Request Recover Right lane");
        srv.request.lane = srv.request.RIGHT;
    } else if (lane == LEFT)
    {
        ROS_DEBUG("Request Recover Left lane");
        srv.request.lane = srv.request.LEFT;
    }

    if (_recoverClient.call(srv))
    {
        ROS_DEBUG("Request Recover Receive results...");
        std::shared_ptr<LineParams> params = nullptr;
        if (srv.response.params.size() > 0)
        {
            ROS_DEBUG_STREAM("Request Recover Receive results OK, size = " << srv.response.params.size());
            auto p = LineParams::ImplType();
            std::copy(srv.response.params.begin(), srv.response.params.end(), p.begin());
            params = std::make_shared<LineParams>(p);
            ROS_DEBUG_STREAM("New params: " << p[0] << ' ' << p[1] << ' ' << p[2]);
        }

        if (srv.request.lane == srv.request.LEFT)
        {
            leftParams = params;
        }
        else if (srv.request.lane == srv.request.RIGHT)
        {
            rightParams = params;
        }
        ROS_DEBUG("OK");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service requestRecover");
        return false;
    }
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