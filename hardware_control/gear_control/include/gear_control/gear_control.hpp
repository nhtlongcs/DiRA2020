#pragma once

#include "JHPWMPCA9685.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <memory>
#include <string>

class DiraGearNode {
 public:
  DiraGearNode(ros::NodeHandle &nh);
  virtual ~DiraGearNode();

 private:
  void throttleCallback(std_msgs::Float32 const &throttle);
  void steerCallback(std_msgs::Float32 const &steering);
  void laserSensorCallback(std_msgs::Bool const &status);
  void move();
  void brake();
  void onInit();
  enum class Direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };
  inline static std::map<Direction, std::string> const dirName{
      {Direction::STOP, "stop"},
      {Direction::FORWARD, "forward"},
      {Direction::BACKWARD, "backward"},
  };

  int pwmFreq_;
  float throttle_ = 0;
  bool laserStatus_ = true;

  Direction direction_ = Direction::STOP;
  std::unique_ptr<PCA9685> pca9685;

  ros::Subscriber speedSub_;
  ros::Subscriber steerSub_;
  ros::Subscriber laserSub_;
};
