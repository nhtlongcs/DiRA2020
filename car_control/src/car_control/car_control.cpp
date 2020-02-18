#include "car_control/car_control.h"
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

CarControl::CarControl()
    : _nh{"car_control"}, _configServer{_nh}
{
    _configServer.setCallback(boost::bind(&CarControl::configCallback, this, _1, _2));
    // cv::namedWindow("PID config", cv::WINDOW_GUI_NORMAL);
    // cv::createTrackbar("kP", "PID config", &kP, 20);
    // cv::createTrackbar("kI", "PID config", &kI, 20);
    // cv::createTrackbar("kD", "PID config", &kD, 20);

    carPos.x = 165;
    carPos.y = 180;
    steer_publisher = _nh.advertise<std_msgs::Float32>("set_angle", 20);
    speed_publisher = _nh.advertise<std_msgs::Float32>("set_speed", 20);
    control_subscriber = _nh.subscribe("control", 1, &CarControl::driveCallback, this);
}

CarControl::~CarControl() {}

void CarControl::configCallback(car_control::CarControlConfig &config, uint32_t level)
{
    kP = config.P;
    kI = config.I;
    kD = config.D;
    carPos.x = config.carpos_x;
    carPos.y = config.carpos_y;
    minVelocity = config.min_velocity;
    maxVelocity = config.max_velocity;
}

void CarControl::driveCallback(const cds_msgs::control& msg)
{

}

int CarControl::getMaxSpeed() const
{
    return maxVelocity;
}

int CarControl::getMinSpeed() const
{
    return minVelocity;
}

float CarControl::errorAngle(const Point &dst)
{
    float X = dst.x - carPos.x;
    float Y = dst.y;
    float angle = atan2(Y, X) * 180.0 / CV_PI - 90.0;
    return angle;
}

void CarControl::driverCar(const Point &cur, float velocity)
{
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;
    // carPos.x = 165;
    float error = -errorAngle(cur);
    //PID controller
    t_kP = error;
    t_kI += error;
    t_kD = error - preError;
    angle.data = (kP * t_kP + kI * t_kI + kD * t_kD) / 1000.;
    // speed.data = fabs(error) < 1 ? maxVelocity : (velocity - fabs(error) * 0.35);

    float newSpeed = velocity;
    if (abs(angle.data) > 40)
    {
        newSpeed = 5;
    }
    else
    {
        newSpeed = velocity * (1 - abs(angle.data) / 45.0f);
    }

    // speed.data = velocity;
    speed.data = newSpeed;
    preError = error;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
}
