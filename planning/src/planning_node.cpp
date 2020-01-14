#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <string>
#include "planning/planning.h"

static constexpr const int RATE = 15;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_node");

    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    std::string path = ros::package::getPath("lane_detect");

    // cv::namedWindow("Threshold");
    laneDetect = new DetectLane();
    objectDetect = new DetectObject(laneDetect);
    car = new CarControl();
    planner = new Planning(laneDetect, objectDetect);

    ros::Rate rate(RATE);
    image_transport::Subscriber sub = it.subscribe("team220/camera/rgb", 1, imageColorCallback);
    image_transport::Subscriber sub2 = it.subscribe("team220/camera/depth", 1, imageDepthCallback);
    image_transport::Subscriber sub_binary = it.subscribe("lane_detect/lane_seg", 1, imageBinaryCallback);
    ros::Subscriber sub_sign = nh.subscribe("/team220/sign", 1, signCallback);

    cv::Point drivePoint = car->getCarPos();
    int driveSpeed = car->getMaxSpeed();

    bool is_lane_ready = false;
    while (nh.getParam("mobilenet_node/ready", is_lane_ready) == false || !is_lane_ready)
    {
        ROS_INFO_ONCE("Waiting for mobilenet ready...");
        rate.sleep();
    }

    while (ros::ok())
    {
        ros::spinOnce();
        planner->planning(drivePoint, driveSpeed, car->getMaxSpeed(), car->getMinSpeed());
        laneDetect->show(car->getCarPos(), &drivePoint);
        if (forceStop)
        {
            car->driverCar(car->getCarPos(), 0);
        }
        else
        {
            car->driverCar(drivePoint, driveSpeed);
        }

        int key = cv::waitKey(1);
        if (key == 32) // press space to force stop
        {
            forceStop = !forceStop;
        }
        else if (key == 'c')
        {
            // capture
        }
        rate.sleep();
    }
    cv::destroyAllWindows();

    delete planner;
}
