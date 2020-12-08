#pragma once

#include "yolov4/yolov4_tiny.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <chrono>

class YoloOnnxTrt {
 public:
  YoloOnnxTrt(ros::NodeHandle const& nh, YoloParams const& params,
              std::string const& subTopic, std::string const& pubTopic);

 private:
  void drawSampleBboxes(std::vector<BBox> const& bboxes);
  void benchmark();
  void readClsName();
  ros::NodeHandle mNh;
  image_transport::ImageTransport mIt;
  image_transport::Subscriber mImgSub;
  image_transport::Publisher mImgPub;
  void imgCallback(sensor_msgs::ImageConstPtr const& msg);
  bool mVisualize = false;

  int curBench = 0;
  int nbBench = 500;
  double runTime = 0;

  int mBatchSize;
  std::vector<std::string> mClsNames;

  YoloV4Tiny yolo;
  cv::Mat mSampleImg;
  std::vector<cv::Mat> mBatchImgs;
};