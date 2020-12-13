#include <ros/ros.h>
#include <ros/console.h>
#include "lane_detect/lane_detect.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detect_node");

    bool is_debug = false;
    if (!ros::param::get("/debug_mode", is_debug))
    {
        ROS_WARN("Cannot get param debug_mode, use default = false");
    }

    if (is_debug)
    {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    LaneDetect lane_detect{is_debug};

    ROS_INFO("lane_detect_node started");
    ros::spin();
}
