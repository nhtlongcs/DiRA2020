#include <ros/ros.h>
#include "lane_detect/lane_detect.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detect_node");
    LaneDetect lane_detect;
    ros::spin();
}
