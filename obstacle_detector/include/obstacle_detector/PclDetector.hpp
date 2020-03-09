#pragma once
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>

#include "obstacle_detector/PclDetectorConfig.h"

using namespace pcl;

class PclDetector {
 public:
  PclDetector(ros::NodeHandle const &nh, ros::NodeHandle const &pnh);
  ~PclDetector() = default;

 private:
  ros::NodeHandle nh, pnh;
  ros::Subscriber pclSub;
  ros::Publisher pclPub, visualPub;
  std::string frameID;
  ros::Time stamp;

  PointCloud<PointXYZ>::Ptr cloud;
  std::vector<PointCloud<PointXYZ>::Ptr> segArr;

  bool isShow = false;
  double scanRange = 2;
  double sacThreshold = .01;
  double minObjHeight = .03;
  double tolerance = .02;

  dynamic_reconfigure::Server<obstacle_detector::PclDetectorConfig> drServer;
  dynamic_reconfigure::Server<
      obstacle_detector::PclDetectorConfig>::CallbackType drServerCB;

  void PreProcess();
  void RemovePlane();
  void ExtractObjs();
  void Visualize();

  void PclCallback(sensor_msgs::PointCloud2ConstPtr const &msg);
  void ReconfigureCB(obstacle_detector::PclDetectorConfig &config,
                     uint32_t level);
};
