#include <ros/ros.h>
#include "object_detect.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detect_node");

    ObjectDetect objectDetect;

    ros::Rate rate{15};
    while (ros::ok())
    {
        ros::spinOnce();
        objectDetect.update();
        rate.sleep();
    }
}
