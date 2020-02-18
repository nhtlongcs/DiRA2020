#include <ros/ros.h>
#include "sign_detect/signdetect.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "signdetect_node");
    SignDetect sign_detect;
    ros::Rate rate{15};

    ROS_INFO("Sign detect init success");
    while (ros::ok())
    {
        ros::spinOnce();
        sign_detect.update();
        rate.sleep();
    }
}