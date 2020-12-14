#include "road_shape_detect/road_shape_detect.hpp"
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

RoadShapeDetector::RoadShapeDetector()
    : it{_nh}
{
    std::string roadSegTopic, roadPubTopic, transportHint, crossroadTopic;
    ROS_ASSERT(ros::param::get("/road_segmentation_topic", roadSegTopic));
    // ROS_ASSERT(ros::param::get("/road", roadPubTopic));
    ROS_ASSERT(ros::param::get("/transport_hint", transportHint));
    ROS_ASSERT(ros::param::get("/crossroad", crossroadTopic));
    
    roadSub = it.subscribe(roadSegTopic, 1, &RoadShapeDetector::roadSegCallback, this, image_transport::TransportHints{transportHint});
    crossroadPub = _nh.advertise<std_msgs::Int8>(crossroadTopic, 1);
}

void RoadShapeDetector::roadSegCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr inImgPtr;
    try {
        inImgPtr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception const& e) {
        ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
        return;
    }
    std_msgs::Int8 crossroad_msg;
    cv::Mat image = inImgPtr->image.clone();
    cv::Rect ROI{0, this->dropTop, image.cols - 1, image.rows - this->dropTop - 1};
    image = image(ROI);
    cv::threshold(image, image, 150, 255, cv::THRESH_BINARY);

    cv::Mat image2 = image.clone();
    // image2 = cv::Scalar{255} - image2;
    cv::floodFill(image2, cv::Point{image.cols / 2, image.rows - 10}, cv::Scalar{0});
    cv::bitwise_xor(image, image2, image);

    cv::imshow("Bottom", image);
    cv::waitKey(1);
    float area = cv::countNonZero(image) * 1.0f / (image.rows * image.cols);
    // ROS_INFO("Road Area %.2f", area);
    crossroad_msg.data = 0;
    
    if (area > minArea)
    {
        // TODO: later 
        crossroad_msg.data = 1;
        // ROS_INFO("Crossroad detected ================================");
    }
    crossroadPub.publish(crossroad_msg);


}
