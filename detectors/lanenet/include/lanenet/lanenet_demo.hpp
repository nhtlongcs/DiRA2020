#pragma once

#include "lanenet/lanenet.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <chrono>

class LaneNetDemo {
 public:
  LaneNetDemo(ros::NodeHandle const& nh, LaneNetParams const& params,
              std::string const& rgbTopic, std::string const& outLaneTopic,
              std::string const& outRoadTopic, image_transport::TransportHints const& transportHint);

 private:
  void benchmark();
  image_transport::ImageTransport mIt;
  image_transport::Subscriber mImgSub;
  image_transport::Publisher mImgPub1;
  image_transport::Publisher mImgPub2;
  void imgCallback(sensor_msgs::ImageConstPtr const& msg);
  bool mVisualize = false;

  int inferCount = 0;
  int curBench = 0;
  int nbBench = 500;
  double runTime = 0;

  int mBatchSize;
  std::vector<std::string> mClsNames;

  LaneNet lanenet;
};