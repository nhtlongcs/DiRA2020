#pragma once

#include "yolov4/yolov4_tiny.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class YoloOnnxTrt {
 public:
  YoloOnnxTrt(ros::NodeHandle const& nh, YoloParams const& params,
              std::string const& subTopic, std::string const& pubTopic);

 private:
  cv::Mat drawSampleBboxes(cv::Mat const& img, std::vector<BBox> const& bboxes);
  void benchmark();
  void readClsName();
  ros::NodeHandle mNh;
  image_transport::ImageTransport mIt;
  image_transport::Subscriber mImgSub;
  image_transport::Publisher mImgPub;
  void imgCallback(sensor_msgs::ImageConstPtr const& msg);
  bool mVisualize = true;

  int inferCount = 0;
  int curBench = 0;
  int nbBench = 500;
  double runTime = 0;

  int mBatchSize;
  std::vector<std::string> mClsNames;

  YoloV4Tiny yolo;
};