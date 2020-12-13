#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <dynamic_reconfigure/server.h>
#include "car_control/CarControlConfig.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

class CarControl
{
public:
    CarControl();

public:
    void configCallback(car_control::CarControlConfig &config, uint32_t level);
    void driveCallback(const geometry_msgs::Twist &msg);

private:
    ros::NodeHandle _nh;
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;
    ros::Subscriber control_subscriber;
    dynamic_reconfigure::Server<car_control::CarControlConfig> _configServer;

private:
    cv::Point carPos;
    float minVelocity = 10;
    float maxVelocity = 30;
    float preError;
    // int kP = 1;
    // int kI = 3;
    // int kD = 10;
    int kCU;
    int PU;
    float kP = 1;
    float kI = 3;
    float kD = 10;
    float t_kP = 0.0;
    float t_kI = 0.0;
    float t_kD = 0.0;

    const int auxTurn = 40;
};
#endif