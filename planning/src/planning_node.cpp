#include <ros/ros.h>
#include <iostream>
#include <string>
#include "planning/planning.h"

static constexpr const int RATE = 15;
static constexpr const char *NODE_NAME = "planning_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    Planning planner;

    ros::Rate rate(RATE);

    ROS_INFO("%s started", NODE_NAME);
    while (ros::ok())
    {
        ros::spinOnce();
        planner.planning();
        rate.sleep();
    }
}
