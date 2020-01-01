#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <cds_msgs/sign.h>
#include "signdetect.h"

static const int RATE = 15;
DetectSign *signDetect;
ros::Publisher pub;

void imageColorCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (!cv_ptr->image.empty())
        {
            signDetect->updateRGB(cv_ptr->image);
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
    cv::Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (!cv_ptr->image.empty())
        {
            cv::cvtColor(cv_ptr->image, out, cv::COLOR_BGR2GRAY);
            signDetect->updateDepth(out);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "signdetect");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    const std::string& packagePath = ros::package::getPath("signdetect");
    const std::string& leftTemplate = packagePath + "/images/left.png";
    const std::string& rightTemplate = packagePath + "/images/right.png";

    const cv::Mat&& LEFT_TEMPLATE = cv::imread(leftTemplate, cv::IMREAD_GRAYSCALE);
    const cv::Mat&& RIGHT_TEMPLATE = cv::imread(rightTemplate, cv::IMREAD_GRAYSCALE);

    signDetect = new DetectSign(LEFT_TEMPLATE, RIGHT_TEMPLATE);

    ros::Rate rate(RATE);

    image_transport::Subscriber sub = it.subscribe("team220/camera/rgb", 1, imageColorCallback);
    image_transport::Subscriber sub2 = it.subscribe("team220/camera/depth", 1, imageDepthCallback);

    pub = nh.advertise<cds_msgs::sign>("team220/sign", 1);

    while (ros::ok())
    {
        ros::spinOnce();
        int sign = signDetect->detect();

        {
            cds_msgs::sign signmsg;
            signmsg.header.stamp = ros::Time::now();
            signmsg.sign_id = sign;
            pub.publish(signmsg);
        }
        cv::waitKey(1);
        rate.sleep();
    }

    delete signDetect;
}