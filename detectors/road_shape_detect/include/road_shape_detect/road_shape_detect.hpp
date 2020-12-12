#pragma once
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>
#include <string>

class RoadShapeDetector
{
public:
    RoadShapeDetector();
    void roadSegCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber roadSub;
    ros::Publisher roadPub;

    int dropTop = 80;
    float minArea = 0.6f;
};