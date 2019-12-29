#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <cds_msgs/sign.h>

#include "detectobject.h"
#include "detectlane.h"
#include "planning.h"
#include "carcontrol.h"

bool STREAM = true;
bool forceStop = false;

// VideoCapture capture("video.avi");
DetectLane *laneDetect;
DetectObject *objectDetect;
Planning *planner;
CarControl *car;
int skipFrame = 1;

static const int RATE = 15;
static int delay = RATE;            // delay 1s
static int countTurning = RATE * 3; // turn in 3s
static int prevSign = 0, sign = 0;

void imageColorCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (!cv_ptr->image.empty())
        {
            planner->updateColor(cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
void imageDepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (!cv_ptr->image.empty())
        {
            planner->updateDepth(cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void imageBinaryCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        if (!cv_ptr->image.empty())
        {
            planner->updateBinary(cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void signCallback(const cds_msgs::sign &msg)
{
    ROS_INFO("SIGN = %d", msg.sign_id);
    planner->updateSign(msg.sign_id);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    std::string path = ros::package::getPath("lane_detect");

    // cv::namedWindow("Threshold");
    laneDetect = new DetectLane();
    objectDetect = new DetectObject();
    car = new CarControl();
    planner = new Planning(laneDetect, objectDetect);

    ros::Rate rate(RATE);

    image_transport::Subscriber sub = it.subscribe("team220/camera/rgb", 1, imageColorCallback);
    image_transport::Subscriber sub2 = it.subscribe("team220/camera/depth", 1, imageDepthCallback);
    image_transport::Subscriber sub_binary = it.subscribe("lane_detect/lane_seg", 1, imageBinaryCallback);
    ros::Subscriber sub_sign = nh.subscribe("team220/sign", 1, signCallback);

    cv::Point drivePoint = car->getCarPos();
    int driveSpeed = car->getMaxSpeed();

    while (ros::ok())
    {
        ros::spinOnce();

        if (forceStop)
        {
            car->driverCar(drivePoint, 0);
        } else
        {
            planner->planning(drivePoint, driveSpeed, car->getMaxSpeed(), car->getMinSpeed());
            laneDetect->show(&drivePoint);
            car->driverCar(drivePoint, driveSpeed);
        }

        int key = cv::waitKey(1);
        if (key == 32) // press space to force stop
        {
            forceStop = !forceStop;
        }

        rate.sleep();
    }
    cv::destroyAllWindows();

    delete car;
    delete laneDetect;
    delete objectDetect;
    delete planner;
}
