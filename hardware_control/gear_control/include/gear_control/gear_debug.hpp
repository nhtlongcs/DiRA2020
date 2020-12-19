#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "gear_control/gearConfig.h"
using cfg_t = gear_control::gearConfig;

class GearDebug {
 private:
  dynamic_reconfigure::Server<cfg_t> drServer;
  void cfgCallback(cfg_t& cfg, uint32_t level);
  ros::Publisher speedPub;
  ros::Publisher steerPub;

  float speed, steer;

 public:
  GearDebug(ros::NodeHandle& nh);
};
