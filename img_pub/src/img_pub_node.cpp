#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_fake");
    ros::NodeHandle nh;
    image_transport::ImageTransport it{nh};

    image_transport::Publisher pub = it.advertise("/team220/camera/rgb", 1);

    ros::Rate rate{30};

    if (argc != 2)
    {
        ROS_FATAL("./program image_path");
        return EXIT_FAILURE;
    }

    cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);

    while (ros::ok())
    {
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            pub.publish(msg);
        }
        rate.sleep();
    }
    return 0;
}