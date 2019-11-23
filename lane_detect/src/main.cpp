#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "detectsign.h"
#include "carcontrol.h"

int cnt;
bool STREAM = true;

// VideoCapture capture("video.avi");
DetectLane * laneDetector;
DetectSign * signDetector;
CarControl *car;
int skipFrame = 1;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if(cv_ptr->image.rows > 0)
        {
            laneDetector->updateRGB(cv_ptr->image);
            signDetector->updateRGB(cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if(cv_ptr->image.rows > 0)
        {
            laneDetector->updateDepth(cv_ptr->image);
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

    cv::namedWindow("Threshold");
    cv::namedWindow("RGB");
    cv::namedWindow("depth");
    cnt = 0;
    laneDetector = new DetectLane();
    signDetector = new DetectSign();
    car = new CarControl();

    ros::Rate rate(20);

    image_transport::Subscriber sub2 = it.subscribe("team1/camera/depth", 1, imageCallback2);
    image_transport::Subscriber sub = it.subscribe("team1/camera/rgb", 1, imageCallback);

    while (ros::ok()) {
        ros::spinOnce();

        laneDetector->detect();

        cv::Point drivePoint = laneDetector->calculateError(car->getCarPos());

        int turn = signDetector->detect();
        int speed = 30;
        if (turn != 0)
        {
            std::cout << "TURN " << turn << std::endl;
            speed = 0;
        }

        drivePoint.x += turn * 120;

        car->driverCar(drivePoint, speed);
        cv::waitKey(1);

        rate.sleep();
    } 
    cv::destroyAllWindows();

    delete signDetector;
    delete car;
    delete laneDetector;
}
