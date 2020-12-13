#include <ros/ros.h>
#include <iostream>
#include <string>
#include <ros/console.h>
#include "planning/planning.h"

static constexpr const int RATE = 15;
static constexpr const char *NODE_NAME = "planning_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    Planning planner;

    if (true)
    {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    ros::Rate rate(RATE);

    ROS_INFO("%s started", NODE_NAME);
    while (ros::ok())
    {
        ros::spinOnce();
        planner.planning();
        rate.sleep();
    }
}
