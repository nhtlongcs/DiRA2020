#include "rplidar_ros/rplidar_ros.hpp"
#include <sensor_msgs/LaserScan.h>

namespace {
static float getAngle(rplidar_response_measurement_node_hq_t const &node) {
  return node.angle_z_q14 * 90.f / 16384.f;
}
double deg2rad(double x) { return x * M_PI / 180; }

}  // namespace

namespace rplidar_ros {
RplidarROS::RplidarROS(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  pnh.param<std::string>("channel_type", channel_type, "serial");
  pnh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  pnh.param<int>(
      "serial_baudrate", serial_baudrate,
      115200 /*256000*/);  // ros run for A1 A2, change to 256000 if A3
  pnh.param<std::string>("frame_id", frame_id, "laser_frame");
  pnh.param<bool>("inverted", inverted, false);
  pnh.param<bool>("angle_compensate", angle_compensate, false);
  pnh.param<std::string>("scan_mode", scan_mode, "Standard");
  scanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

  ROS_INFO_STREAM("SDK Version: " << RPLIDAR_SDK_VERSION);

  if (!connectDriver()) {
    throw std::runtime_error("RPLIDAR driver error");
  }

  stop_motor_service =
      nh.advertiseService("stop_motor", &RplidarROS::stop_motor, this);
  start_motor_service =
      nh.advertiseService("start_motor", &RplidarROS::start_motor, this);

  device->startMotor();

  if (!initSpecs()) {
    throw std::runtime_error("RPLIDAR specs error");
  }
}

void RplidarROS::pollOnce() {
  rplidar_response_measurement_node_hq_t nodes[360 * 8];
  size_t count = sizeof(nodes) / sizeof(nodes[0]);

  start_scan_time = ros::Time::now();
  op_result = device->grabScanDataHq(nodes, count);
  scan_duration = (ros::Time::now() - start_scan_time).toSec();

  if (op_result == RESULT_OK) {
    op_result = device->ascendScanData(nodes, count);
    float angle_min = deg2rad(0.0f);
    float angle_max = deg2rad(359.0f);
    if (op_result == RESULT_OK) {
      if (angle_compensate) {
        // const int angle_compensate_multiple = 1;
        int angle_compensate_offset = 0;
        rplidar_response_measurement_node_hq_t
            angle_compensate_nodes[angle_compensate_nodes_count];
        memset(angle_compensate_nodes, 0,
               angle_compensate_nodes_count *
                   sizeof(rplidar_response_measurement_node_hq_t));

        for (int i = 0; i < count; ++i) {
          if (nodes[i].dist_mm_q2 != 0) {
            float angle = getAngle(nodes[i]);
            int angle_value = (int)(angle * angle_compensate_multiple);
            if ((angle_value - angle_compensate_offset) < 0)
              angle_compensate_offset = angle_value;
            for (int j = 0; j < angle_compensate_multiple; ++j) {
              int angle_compensate_nodes_index =
                  angle_value - angle_compensate_offset + j;
              if (angle_compensate_nodes_index >= angle_compensate_nodes_count)
                angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
              angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
            }
          }
        }
        publishScan(angle_compensate_nodes, angle_compensate_nodes_count,
                    angle_min, angle_max);
      } else {
        int start_node = 0, end_node = 0;
        int i = 0;
        // find the first valid node and last valid node
        while (nodes[i++].dist_mm_q2 == 0)
          ;
        start_node = i - 1;
        i = count - 1;
        while (nodes[i--].dist_mm_q2 == 0)
          ;
        end_node = i + 1;
        angle_min = deg2rad(getAngle(nodes[start_node]));
        angle_max = deg2rad(getAngle(nodes[end_node]));
        publishScan(&nodes[start_node], end_node - start_node + 1, angle_min,
                    angle_max);
      }
    } else if (op_result == RESULT_OPERATION_FAIL) {
      // All the data is invalid, just publish them
      float angle_min = deg2rad(0.0f);
      float angle_max = deg2rad(359.0f);
      publishScan(nodes, count, angle_min, angle_max);
    }
  }
}

void RplidarROS::publishScan(
    rplidar_response_measurement_node_hq_t const *const nodes,
    size_t node_count, float angle_min, float angle_max) {
  sensor_msgs::LaserScanPtr scanMsg(new sensor_msgs::LaserScan);
  scanMsg->header.stamp = start_scan_time;
  scanMsg->header.frame_id = frame_id;

  bool reversed = angle_min < angle_max;
  if (reversed) {
    scanMsg->angle_min = M_PI - angle_max;
    scanMsg->angle_max = M_PI - angle_min;
  } else {
    scanMsg->angle_min = M_PI - angle_min;
    scanMsg->angle_max = M_PI - angle_max;
  }
  scanMsg->angle_increment =
      (scanMsg->angle_max - scanMsg->angle_min) / (node_count - 1);

  scanMsg->scan_time = scan_duration;
  scanMsg->time_increment = scan_duration / (node_count - 1);
  scanMsg->range_min = 0.15;
  scanMsg->range_max = max_distance;  // 8.0;

  scanMsg->intensities.resize(node_count);
  scanMsg->ranges.resize(node_count);
  bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
  if (!reverse_data) {
    for (size_t i = 0; i < node_count; i++) {
      float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
      if (read_value == 0.0)
        scanMsg->ranges[i] = std::numeric_limits<float>::infinity();
      else
        scanMsg->ranges[i] = read_value;
      scanMsg->intensities[i] = (float)(nodes[i].quality >> 2);
    }
  } else {
    for (size_t i = 0; i < node_count; i++) {
      float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
      if (read_value == 0.0)
        scanMsg->ranges[node_count - 1 - i] =
            std::numeric_limits<float>::infinity();
      else
        scanMsg->ranges[node_count - 1 - i] = read_value;
      scanMsg->intensities[node_count - 1 - i] = (float)(nodes[i].quality >> 2);
    }
  }
  scanPub.publish(scanMsg);
}

bool RplidarROS::connectDriver() {
  device = std::unique_ptr<rp::RPlidarDriver>(
      rp::RPlidarDriver::CreateDriver(rp::DRIVER_TYPE_SERIALPORT));
  if (!device) {
    ROS_FATAL_STREAM("Could not create driver");
    return false;
  }
  // make connection...
  if (IS_FAIL(
          device->connect(serial_port.c_str(), uint32_t(serial_baudrate)))) {
    ROS_FATAL_STREAM("Cannot bind to the specified serial port "
                     << serial_port);
    rp::RPlidarDriver::DisposeDriver(device.get());
    return false;
  }
  // get rplidar device info
  if (!getRPLIDARDeviceInfo()) {
    ROS_FATAL_STREAM("Cannot get device info");
    return false;
  }
  // check health...
  if (!checkRPLIDARHealth()) {
    ROS_FATAL_STREAM("Cannot get device health");
    rp::RPlidarDriver::DisposeDriver(device.get());
    return false;
  }
  return true;
}

bool RplidarROS::initSpecs() {
  rp::RplidarScanMode current_scan_mode;
  if (scan_mode.empty()) {
    op_result = device->startScan(false, true, 0, &current_scan_mode);
  } else {
    std::vector<rp::RplidarScanMode> allSupportedScanModes;
    op_result = device->getAllSupportedScanModes(allSupportedScanModes);

    if (IS_OK(op_result)) {
      _u16 selectedScanMode = _u16(-1);
      for (auto it = allSupportedScanModes.begin();
           it != allSupportedScanModes.end(); ++it) {
        if (it->scan_mode == scan_mode) {
          selectedScanMode = it->id;
          break;
        }
      }
      if (selectedScanMode == _u16(-1)) {
        ROS_ERROR_STREAM("scan mode \""
                         << scan_mode
                         << "\" is not supported by lidar, supported modes:");
        for (auto iter = allSupportedScanModes.begin();
             iter != allSupportedScanModes.end(); ++iter) {
          ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK",
                    iter->scan_mode, iter->max_distance,
                    (1000 / iter->us_per_sample));
        }
        op_result = RESULT_OPERATION_FAIL;
      } else {
        op_result =
            device->startScanExpress(false /* not force scan */,
                                     selectedScanMode, 0, &current_scan_mode);
      }
    }
  }

  if (!IS_OK(op_result)) {
    ROS_ERROR("Cannot start scan: %08x!", op_result);
    return false;
  }

  // default frequent is 10 hz (by motor pwm value),
  // current_scan_mode.us_per_sample is the number of scan point per us
  angle_compensate_multiple =
      1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0;
  if (angle_compensate_multiple < 1) {
    angle_compensate_multiple = 1;
  }
  angle_compensate_nodes_count = 360 * angle_compensate_multiple;

  max_distance = current_scan_mode.max_distance;
  ROS_INFO_STREAM("scan mode: " << current_scan_mode.scan_mode << std::fixed
                                << std::setprecision(1) << ", max_distance: "
                                << current_scan_mode.max_distance
                                << " m, point number: "
                                << (1000 / current_scan_mode.us_per_sample)
                                << "K, "
                                   "angle_compensate: "
                                << angle_compensate_multiple);
  return true;
}

bool RplidarROS::getRPLIDARDeviceInfo() {
  u_result op_result;
  rplidar_response_device_info_t devinfo;

  op_result = device->getDeviceInfo(devinfo);
  if (IS_FAIL(op_result)) {
    if (op_result == RESULT_OPERATION_TIMEOUT) {
      ROS_ERROR("Operation time out. RESULT_OPERATION_TIMEOUT!");
    } else {
      ROS_ERROR("Unexpected error, code: %x", op_result);
    }
    return false;
  }

  std::stringstream ss;
  ss << "RPLIDAR S/N: 0x" << std::hex << std::uppercase;
  for (int pos = 0; pos < 16; ++pos) {
    ss << int(devinfo.serialnum[pos]);
  }
  ROS_INFO_STREAM(ss.str());
  ROS_INFO_STREAM("Firmware ver: " << (devinfo.firmware_version >> 8) << "."
                                   << (devinfo.firmware_version & 0xFF));
  ROS_INFO_STREAM("Hardware Rev: " << int(devinfo.hardware_version));
  return true;
}

bool RplidarROS::checkRPLIDARHealth() {
  u_result op_result;
  rplidar_response_device_health_t healthinfo;
  op_result = device->getHealth(healthinfo);
  if (IS_OK(op_result)) {
    ROS_INFO_STREAM("RPLidar health status: " << int(healthinfo.status));
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
      ROS_ERROR_STREAM(
          "Error, rplidar internal error detected. Please reboot the device to "
          "retry");
      return false;
    } else {
      return true;
    }
  } else {
    ROS_ERROR("Error, cannot retrieve rplidar health code: %x", op_result);
    return false;
  }
}

bool RplidarROS::start_motor(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &res) {
  if (!device) {
    return false;
  }
  if (device->isConnected()) {
    ROS_DEBUG("Start motor");
    device->startMotor();
    device->startScan(0, 1);
  } else
    ROS_INFO("lost connection");
  return true;
}

bool RplidarROS::stop_motor(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &res) {
  if (!device) {
    return false;
  }
  ROS_DEBUG("Stop motor");
  device->stopMotor();
  return true;
}

RplidarROS::~RplidarROS() {
  if (device) {
    device->stopMotor();
    device->stop();
    rp::RPlidarDriver::DisposeDriver(device.get());
  }
  ROS_INFO("RPLIDAR off");
}

}  // namespace rplidar_ros
