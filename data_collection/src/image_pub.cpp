#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detect_node");

    ros::NodeHandle nh;
    image_transport::ImageTransport it{nh};

    std::string image_topic;
    ROS_ASSERT(ros::param::get("/rgb_topic", image_topic));
    image_transport::Publisher pub = it.advertise(image_topic, 1);

    std::string path = ros::package::getPath("data_collection") + "/images/test.png";
    cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);

    cv_bridge::CvImage outImg;
    outImg.encoding = "bgr8";
    outImg.image = image;

    ros::Rate rate{15};

    while (ros::ok())
    {
        ros::spinOnce();
        pub.publish(outImg.toImageMsg());
        rate.sleep();
    }

    return 0;
}
