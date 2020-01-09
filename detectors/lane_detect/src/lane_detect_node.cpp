#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <string>

#include "lane_detect.h"

ros::Subscriber mobilenet_sub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detect_node");

    LaneDetect lane_detect;
    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);

    // // cv::namedWindow("Threshold");
    // laneDetect = new DetectLane();
    // image_transport::Subscriber sub = it.subscribe("team220/camera/rgb", 1, imageColorCallback);
    // image_transport::Subscriber sub2 = it.subscribe("team220/camera/depth", 1, imageDepthCallback);
    // image_transport::Subscriber sub_binary = it.subscribe("lane_detect/lane_seg", 1, imageBinaryCallback);
    // ros::Subscriber sub_sign = nh.subscribe("/team220/sign", 1, signCallback);

    // cv::Point drivePoint = car->getCarPos();
    // int driveSpeed = car->getMaxSpeed();

    // bool is_lane_ready = false;
    // while (nh.getParam("mobilenet_node/ready", is_lane_ready) == false || !is_lane_ready)
    // {
    //     ROS_INFO_ONCE("Waiting for mobilenet ready...");
    //     rate.sleep();
    // }

    ros::Rate rate(15);
    while (ros::ok())
    {
        ros::spinOnce();
        lane_detect.update();
        rate.sleep();
    }
}
