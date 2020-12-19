#pragma once

#include "sdk/rplidar.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <memory>
#include <string>

namespace rplidar_ros {

class RplidarROS {
 public:
  RplidarROS(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  virtual ~RplidarROS();
  void pollOnce();

 private:
  bool connectDriver();
  bool initSpecs();
  bool getRPLIDARDeviceInfo();
  bool checkRPLIDARHealth();

  void publishScan(rplidar_response_measurement_node_hq_t const *const nodes,
                   size_t node_count, float angle_min, float angle_max);

  bool start_motor(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res);
  bool stop_motor(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res);
  ros::ServiceServer stop_motor_service;
  ros::ServiceServer start_motor_service;
  ros::Publisher scanPub;
  ros::Time start_scan_time;
  double scan_duration;
  u_result op_result;

  std::string channel_type;
  std::string serial_port;
  std::string scan_mode;
  std::string frame_id;
  bool inverted = false;
  bool angle_compensate = true;
  int serial_baudrate = 115200;
  double max_distance = 8.0;
  int angle_compensate_multiple;
  int angle_compensate_nodes_count;

  std::unique_ptr<rp::RPlidarDriver> device;
};

}  // namespace rplidar_ros