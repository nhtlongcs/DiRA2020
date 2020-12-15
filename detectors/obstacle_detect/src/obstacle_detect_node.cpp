#include "obstacle_detect/obstacle_detect.hpp"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detect", ros::init_options::NoRosout);
  ros::NodeHandle nh, pnh("~");

  try {
    obstacle_detect::ObstacleDetect od(nh);
    ros::spin();
  } catch (std::string const& ex) {
    ROS_FATAL_STREAM("[Obstacle Detect] " << ex);
  }

  return 0;
}
