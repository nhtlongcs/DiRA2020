#ifndef CARCONTROL_H
#define CARCONTROL_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cds_msgs/control.h>
#include <dynamic_reconfigure/server.h>
#include "car_control/CarControlConfig.h"

#include <vector>
#include <cmath>
#include <iostream>

class CarControl
{
public:
    CarControl();
    ~CarControl();
    void driverCar(const cv::Point &cur, float velocity);
    int getMaxSpeed() const;
    int getMinSpeed() const;
    void steerCamera(float angle);
    cv::Point getCarPos() const;

public:
    void configCallback(car_control::CarControlConfig &config, uint32_t level);
    void driveCallback(const cds_msgs::control& msg);

private:
    ros::NodeHandle _nh;
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;
    ros::Publisher cam_publisher;
    ros::Subscriber control_subscriber;

    dynamic_reconfigure::Server<car_control::CarControlConfig> _configServer;

    cv::Point carPos;
    float errorAngle(const cv::Point &dst);
    float minVelocity = 10;
    float maxVelocity = 30;
    float preError;
    int kP = 1;
    int kI = 3;
    int kD = 10;
    float t_kP = 0.0;
    float t_kI = 0.0;
    float t_kD = 0.0;

    const int auxTurn = 40;
};
#endif
