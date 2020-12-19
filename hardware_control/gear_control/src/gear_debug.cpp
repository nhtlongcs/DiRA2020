#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32.h>

#include "gear_control/gear_debug.hpp"

GearDebug::GearDebug(ros::NodeHandle& nh) {
  std::string speedTopic, steerTopic;
  ros::param::param<std::string>("~speed_topic", speedTopic, "set_speed");
  ros::param::param<std::string>("~steer_topic", steerTopic, "set_angle");
  speedPub = nh.advertise<std_msgs::Float32>(speedTopic, 5);
  steerPub = nh.advertise<std_msgs::Float32>(steerTopic, 5);

  drServer.setCallback([&](auto&& cfg, auto&& level) {
    cfgCallback(std::forward<decltype(cfg)>(cfg),
                std::forward<decltype(level)>(level));
  });
}

void GearDebug::cfgCallback(cfg_t& cfg, uint32_t level) {
  std_msgs::Float32 msg;
  if (speed != cfg.speed) {
    msg.data = speed = cfg.speed;
    speedPub.publish(msg);
  }
  if (steer != cfg.steer) {
    msg.data = steer = cfg.steer;
    steerPub.publish(msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gear_debug");
  ros::NodeHandle nh;

  try {
    ROS_INFO("[Gear Debug] Initializing node");
    GearDebug gd(nh);
    ros::spin();
  } catch (std::string const& ex) {
    ROS_FATAL_STREAM("[Gear Debug] " << ex);
  }

  return 0;
}