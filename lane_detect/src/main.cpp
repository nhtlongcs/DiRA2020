#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"

int cnt;
bool STREAM = true;

// VideoCapture capture("video.avi");
DetectLane *detect;
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
            detect->updateRGB(cv_ptr->image);
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
            detect->updateDepth(cv_ptr->image);
        } 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// void videoProcess()
// {
//     Mat src;
//     while (true)
//     {
//         // capture >> src;
//         if (src.empty()) break;
//         imshow("View", src);
//         detect->calculateError(src);
//         waitKey(30);
//     }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    
    cv::Mat image = cv::imread("/home/ken/input2.jpg", cv::IMREAD_ANYCOLOR);
    std::cout << "OK";

    cv::namedWindow("Threshold");
    detect = new DetectLane();
    detect->updateRGB(image);

    detect->calculateError();
    // ros::init(argc, argv, "image_listener");
    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);

    // cv::namedWindow("Threshold");
    // cv::namedWindow("RGB");
    // cv::namedWindow("depth");
    // cnt = 0;
    // detect = new DetectLane();
    // car = new CarControl();
    // ros::Rate r(10);

    // image_transport::Subscriber sub2 = it.subscribe("team1/camera/depth", 1, imageCallback2);
    // image_transport::Subscriber sub = it.subscribe("team1/camera/rgb", 1, imageCallback);

    // while (ros::ok()) {
    //     ros::spinOnce();

    //     detect->processDepth();
    //     car->driverCar(detect->calculateError(), 30);
    //     cv::waitKey(1);
    // } 
    // cv::destroyAllWindows();
}
