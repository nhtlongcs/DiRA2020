#include <ros/ros.h>
#include "car_control/car_control.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "car_control_node");
    CarControl car_control;
    ros::spin();
    return 0;
}