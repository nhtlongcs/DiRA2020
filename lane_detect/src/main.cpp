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
bool ready = false;

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

ros::Subscriber mobilenet_sub;

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
            cv::cvtColor(cv_ptr->image, out, cv::COLOR_BGR2GRAY);
            planner->updateDepth(out);
            objectDetect->updateDepth(out);
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
            laneDetect->updateBinary(cv_ptr->image);
            objectDetect->updateBinary(cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void signCallback(const cds_msgs::sign &msg)
{
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
    objectDetect = new DetectObject(laneDetect);
    car = new CarControl();
    planner = new Planning(laneDetect, objectDetect);

    ros::Rate rate(RATE);

    image_transport::Subscriber sub = it.subscribe("team220/camera/rgb", 1, imageColorCallback);
    image_transport::Subscriber sub2 = it.subscribe("team220/camera/depth", 1, imageDepthCallback);
    image_transport::Subscriber sub_binary = it.subscribe("lane_detect/lane_seg", 1, imageBinaryCallback);
    ros::Subscriber sub_sign = nh.subscribe("team220/sign", 1, signCallback);

    cv::Point drivePoint = car->getCarPos();
    int driveSpeed = car->getMaxSpeed();

    bool is_lane_ready = false;
    while (nh.getParam("mobilenet_node/ready", is_lane_ready) == false || !is_lane_ready)
    {
        ROS_INFO("Waiting for mobilenet ready...");
        rate.sleep();
    }

    while (ros::ok())
    {
        ros::spinOnce();
        planner->planning(drivePoint, driveSpeed, car->getMaxSpeed(), car->getMinSpeed());
        laneDetect->show(&drivePoint);
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
        } else if (key == 'c')
        {
            // capture
        }
        rate.sleep();
    }
    cv::destroyAllWindows();

    delete car;
    delete laneDetect;
    delete objectDetect;
    delete planner;
}
