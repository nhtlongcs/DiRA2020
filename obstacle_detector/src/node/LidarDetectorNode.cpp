#include "obstacle_detector/LidarDetector.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "LidarDetectorNode", ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try {
    ROS_INFO("[Lidar Extractor]: Initializing node");
    LidarDetector ld(nh, nh_local);
    ros::spin();
  } catch (char const* s) {
    ROS_FATAL_STREAM("[Lidar Extractor]: " << s);
  } catch (...) {
    ROS_FATAL_STREAM("[Lidar Extractor]: Unexpected error");
  }
  return 0;
}
