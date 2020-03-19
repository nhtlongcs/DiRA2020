#include "obstacle_detector/PclDetector.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "PclDetectorNode", ros::init_options::NoRosout);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    ROS_INFO("[PCL Detect]: Initializing node");
    PclDetector pd(nh, pnh);
    ros::spin();
  } catch (char const* s) {
    ROS_FATAL_STREAM("[PCL Detect]: " << s);
  } catch (...) {
    ROS_FATAL_STREAM("[PCL Detect]: Unexpected error");
  }
  return 0;
}
