#pragma once

#include "Hal.h"
#include "LCDI2C.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace dira_lcd {
class LcdRos {
 public:
  LcdRos(ros::NodeHandle &nh);

 private:
  void lcdCallback(std_msgs::StringConstPtr const &msg);
  void setCursor(int x, int y);
  void print(const char *c);
  ros::Subscriber lcdSub;
  I2C i2c_device;
  LCDI2C lcd;
};
}  // namespace dira_lcd