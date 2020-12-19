#include "rplidar_ros/rplidar_ros.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rplidar_ros", ros::init_options::NoRosout);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    ROS_INFO("[RPLIDAR] Initializing node");
    rplidar_ros::RplidarROS rplidar(nh, pnh);
    while (ros::ok()) {
      rplidar.pollOnce();
      ros::spinOnce();
    }
  } catch (std::string const& ex) {
    ROS_FATAL_STREAM("[RPLIDAR] " << ex);
  }

  return 0;
}