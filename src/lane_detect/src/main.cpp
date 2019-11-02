#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"

int cnt;

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
	    waitKey(1);
        cnt++;
        if (cnt % 10 == 0)
        {        
            std::cout<< "saved" << std::endl;
            std::string savingName = "/home/ken/Desktop/frame-new/im-" + std::to_string(cnt) + ".jpg";
            cv::imwrite(savingName, cv_ptr->image);
        }
        car->driverCar(detect->calculateError(cv_ptr->image), 35);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    
    cv::namedWindow("Threshold");
    cnt = 0;
    detect = new DetectLane();
    car = new CarControl();
	
    cv::startWindowThread();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("team1/camera/rgb", 1, imageCallback);
    ros::spin();
    
    cv::destroyAllWindows();
}
