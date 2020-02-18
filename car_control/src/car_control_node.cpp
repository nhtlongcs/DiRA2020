#include <ros/ros.h>
#include "car_control/car_control.h"

static constexpr const char* NODE_NAME = "car_control_node";

int main(int argc, char* argv[])
{
    ros::init(argc, argv, NODE_NAME);
    CarControl car_control;
    ROS_INFO("%s started", NODE_NAME);
    ros::spin();
    return 0;
}