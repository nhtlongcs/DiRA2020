#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <string>

cv::Mat colorImage;
cv::Mat depthImage;
cv::Ptr<cv::BackgroundSubtractor> pBackSub;
int minDepth = 0;
int maxDepth = 1;
int ratio = 40;
int frame_count = 0;
bool is_stop_writing = true;
std::string path;
std::ofstream file;


void colorCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (!cv_ptr->image.empty())
        {
            colorImage = cv_ptr->image.clone();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to '%s'.", msg->encoding.c_str(), sensor_msgs::image_encodings::MONO8.c_str());
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        if (!cv_ptr->image.empty())
        {
            depthImage = cv_ptr->image.clone();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to '%s'.", msg->encoding.c_str(), sensor_msgs::image_encodings::TYPE_16UC1.c_str());
    }
}

void process()
{
    if (colorImage.empty())
    {
        return;
    }
    if (depthImage.empty())
    {
        return;
    }
    cv::resize(colorImage, colorImage, cv::Size{320, 240});
    cv::resize(depthImage, depthImage, cv::Size{320, 240});
    // cv::imshow("ColorRaw", colorImage);
    // cv::imshow("DepthRaw", depthImage);

    cv::Mat depthThresholded;
    cv::inRange(depthImage, cv::Scalar{minDepth}, cv::Scalar{maxDepth}, depthThresholded);
    // cv::imshow("DepthThresholded", depthThresholded);

    cv::Mat result;
    colorImage.copyTo(result, depthThresholded);
    cv::imshow("Result", result);

    cv::Rect boundingBox = cv::boundingRect(depthThresholded);
    boundingBox.x -= 10;
    boundingBox.y -= 10;
    boundingBox.width += 10;
    boundingBox.height += 10;

    cv::Rect originalRect{0,0,colorImage.cols, colorImage.rows};
    boundingBox = boundingBox & originalRect;

    if (boundingBox.area() <= 0)
    {
        return;
    }

    float area = cv::countNonZero(depthThresholded) * 1.0f / boundingBox.area();
    ROS_INFO_COND(!is_stop_writing, "Area = %.2f", area);
    if (area < (ratio / 100.0f))
    {
        ROS_ERROR_COND(!is_stop_writing, "Drop frame due to area = %.2f < %.2f", area, ratio / 100.0f);
    }
    else if (!is_stop_writing)
    {
        std::string filename = path + "/frame_" + std::to_string(frame_count) + std::string{".jpg"};
        // write image
        if (!cv::imwrite(filename, colorImage))
        {
            ROS_ERROR_COND(!is_stop_writing, "Cannot write image: %s", filename.c_str());
        }
        // write box
        file << filename << ',' << boundingBox.x << ',' << boundingBox.y 
             << ',' << boundingBox.width << ',' << boundingBox.height << std::endl;

        frame_count++;
    }
    
    cv::rectangle(colorImage, boundingBox, cv::Scalar{0,0,255}, 2);
    cv::imshow("Detection", colorImage);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detect_node");

    cv::namedWindow("Trackbar");
    cv::createTrackbar("MinDepth", "Trackbar", &minDepth, 5000);
    cv::createTrackbar("MaxDepth", "Trackbar", &maxDepth, 5000);
    cv::createTrackbar("Ratio", "Trackbar", &ratio, 100);

    ros::NodeHandle nh;
    ros::NodeHandle private_nh{"~"};
    image_transport::ImageTransport it{nh};

    image_transport::Subscriber colorSub = it.subscribe("/camera/rgb/image_raw", 1, &colorCallback);
    image_transport::Subscriber depthSub = it.subscribe("/camera/depth/image", 1, &depthCallback);
    pBackSub = cv::createBackgroundSubtractorMOG2();

    std::string classname;
    const std::unordered_set<std::string> valid_names = {"forward", "left", "right", "notleft", "notright", "stop"};
    if (!private_nh.getParam("class", classname) || valid_names.find(classname) == valid_names.end())
    {
        ROS_FATAL("INVALID NAME");
        exit(EXIT_FAILURE);
    }

    path = ros::package::getPath("data_collection") + "/images/" + classname;
    file = std::ofstream{path + "/bounding_box.csv", std::ofstream::out | std::ofstream::app};
    if (!private_nh.getParam("frame_count", frame_count))
    {
        ROS_WARN("frame_count start at 0");
    }
    else
    {
        ROS_INFO("Start at frame_count = %d", frame_count);
    }
    ros::Rate rate{15};

    ROS_INFO("data_collection_node started");
    ROS_INFO("Press SPACE to start writing images...");
    while (ros::ok())
    {
        ros::spinOnce();
        process();
        if (cv::waitKey(1) == 32)
        {
            is_stop_writing = !is_stop_writing;
            ROS_INFO_COND(is_stop_writing, "Stop writing images");
        }
        rate.sleep();
    }

    file.close();
    return 0;
}
