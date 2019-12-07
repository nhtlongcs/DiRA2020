#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

#include "detectobject.h"
#include "detectsign.h"
#include "detectlane.h"
#include "planning.h"
#include "carcontrol.h"

bool STREAM = true;

// VideoCapture capture("video.avi");
DetectLane* laneDetect;
DetectObject* objectDetect;
DetectSign* signDetect;
Planning *planner;
CarControl *car;
int skipFrame = 1;

static const int RATE = 15;
static int delay = RATE; // delay 1s
static int countTurning = RATE * 3; // turn in 3s
static int prevSign = 0, sign = 0;

void imageColorCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if(!cv_ptr->image.empty())
        {
            planner->updateColor(cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if(!cv_ptr->image.empty())
        {
            planner->updateDepth(cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    std::string path = ros::package::getPath("lane_detect");
    std::string left_path = path + "/images/left.png";
    std::string right_path = path + "/images/right.png";

    const cv::Mat LEFT_TEMPLATE = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
    const cv::Mat RIGHT_TEMPLATE = cv::imread(right_path, cv::IMREAD_GRAYSCALE);

    if (LEFT_TEMPLATE.empty())
    {
        ROS_ERROR("Left sign not found");
    }

    if (RIGHT_TEMPLATE.empty())
    {
        ROS_ERROR("Right sign not found");
    }


    cv::namedWindow("Threshold");
    cv::namedWindow("RGB");
    cv::namedWindow("depth");
    laneDetect = new DetectLane();
    signDetect = new DetectSign(LEFT_TEMPLATE, RIGHT_TEMPLATE);
    objectDetect = new DetectObject();
    car = new CarControl();
    planner = new Planning(laneDetect, objectDetect, signDetect, RATE);

    ros::Rate rate(RATE);

    image_transport::Subscriber sub = it.subscribe("team1/camera/rgb", 1, imageColorCallback);
    image_transport::Subscriber sub2 = it.subscribe("team1/camera/depth", 1, imageDepthCallback);

    cv::Point drivePoint = car->getCarPos();
    int driveSpeed = car->getMaxSpeed();

    while (ros::ok()) {
        ros::spinOnce();

        planner->planning(drivePoint, driveSpeed, car->getMaxSpeed(), car->getMinSpeed());
        laneDetect->show(&drivePoint);
        car->driverCar(drivePoint, driveSpeed);
        cv::waitKey(1);

        rate.sleep();
    } 
    cv::destroyAllWindows();

    delete car;
    delete laneDetect;
    delete objectDetect;
    delete signDetect;
    delete planner;
}
