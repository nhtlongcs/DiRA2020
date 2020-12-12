#include <ros/ros.h>
#include "road_shape_detect/road_shape_detect.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_shape_detect_node");
    RoadShapeDetector detector;
    ros::spin();
    return 0;
}