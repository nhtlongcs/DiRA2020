#include "gear_control/gear_control.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gear_control");
  ros::NodeHandle nh;

  try {
    ROS_INFO("[Gear Control] Initializing node");
    DiraGearNode gear(nh);
    ros::Rate r(2);
    ros::spin();
  } catch (std::string const& ex) {
    ROS_FATAL_STREAM("[Gear Control] " << ex);
  }

  return 0;
}
