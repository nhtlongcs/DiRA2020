#pragma once

#include "lanenet/lanenet.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/SetBool.h>
#include <chrono>

class LaneNetDemo {
 public:
  LaneNetDemo(ros::NodeHandle& nh, LaneNetParams const& params,
              std::string const& rgbTopic, std::string const& outLaneTopic,
              std::string const& outRoadTopic, image_transport::TransportHints const& transportHint);

 private:
  bool setRoadUseDeepHandler(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
  bool setLaneUseDeepHandler(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);

 private:
  void benchmark();
  image_transport::ImageTransport mIt;
  image_transport::Subscriber mImgSub;
  image_transport::Publisher mImgPub1;
  image_transport::Publisher mImgPub2;
  void imgCallback(sensor_msgs::ImageConstPtr const& msg);
  cv::Mat roadImageProc(const cv::Mat& image);
  cv::Mat laneImageProc(const cv::Mat& image);
  bool mVisualize = false;

  int inferCount = 0;
  int curBench = 0;
  int nbBench = 500;
  double runTime = 0;

  int mBatchSize;
  std::vector<std::string> mClsNames;

  LaneNet lanenet;
  bool isRoadUseDeep;
  bool isLaneUseDeep;
  ros::ServiceServer _srvRoad, _srvLane;
};