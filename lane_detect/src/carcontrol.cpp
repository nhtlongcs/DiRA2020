#include "carcontrol.h"

#define MAX_SPEED 40
#define MIN_SPEED 10

CarControl::CarControl()
{

    cv::namedWindow("PID config", cv::WINDOW_GUI_NORMAL);
    cv::createTrackbar("kP", "PID config", &kP, 20);
    cv::createTrackbar("kI", "PID config", &kI, 20);
    cv::createTrackbar("kD", "PID config", &kD, 20);
    
    carPos.x = 165;
    carPos.y = 180;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("hcmus_avengers/set_angle",20);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("hcmus_avengers/set_speed",20);
    cam_publisher = node_obj1.advertise<std_msgs::Float32>("hcmus_avengers/set_camera_angle",20);
}

CarControl::~CarControl() {}

cv::Point CarControl::getCarPos() const
{
    return this->carPos;
}

int CarControl::getMaxSpeed() const
{
    return MAX_SPEED;
}

int CarControl::getMinSpeed() const
{
    return MIN_SPEED;
}

float CarControl::errorAngle(const Point &dst)
{
    float X = dst.x - carPos.x;
    float Y = dst.y;
    float angle = atan2(Y, X) * 180.0 / CV_PI - 90.0;
    return angle;
}

void CarControl::steerCamera(float angle)
{
    std_msgs::Float32 angleMsg;
    angleMsg.data = angle;
    cam_publisher.publish(angleMsg);
}

void CarControl::driverCar(const Point& cur, float velocity)
{
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;
    carPos.x = 165;   
    float error = -errorAngle(cur);
    //PID controller
    t_kP = error;
    t_kI += error;
    t_kD = error - preError;
    angle.data = kP * t_kP + (kI/100000.) * t_kI + kD * t_kD;
    // speed.data = fabs(error) < 1 ? maxVelocity : (velocity - fabs(error) * 0.35);
    speed.data = velocity;
    preError = error;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 
