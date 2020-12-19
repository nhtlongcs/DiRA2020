#include "lcd_ros.h"

#include <iostream>
#include <thread>
#include <ros/param.h>

namespace dira_lcd {

LcdRos::LcdRos(ros::NodeHandle& nh) {
  i2c_device.m_i2c_bus = 1;

  std::string i2c_add;
  ros::param::param<std::string>("lcd_i2c_adr", i2c_add, "38");

  size_t nbCharRead;
  int i2c_add_dec = std::stoi(i2c_add, &nbCharRead, 16);
  ROS_INFO_STREAM("Setting I2C address to " << i2c_add
                                            << " in decimal: " << i2c_add_dec);
  uint8_t addr = uint8_t(i2c_add_dec);

  if (!i2c_device.HALOpen()) {
    std::cerr << "Cannot open I2C peripheral" << std::endl;
    exit(-1);
  }
  std::cout << "I2C peripheral is opened" << std::endl;

  uint8_t data;
  if (!i2c_device.HALRead(addr, 0xFF, 0, &data, "")) {
    throw std::runtime_error("LCD is not connected");
  }
  std::cout << "LCD is connected" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  lcd.LCDInit(&i2c_device, addr, 20, 4);
  lcd.LCDBacklightOn();
  lcd.LCDCursorOn();

  std::string lcdTopic;
  ros::param::param<std::string>("lcd_topic", lcdTopic, "lcd_print");
  lcdSub = nh.subscribe(lcdTopic, 10, &LcdRos::lcdCallback, this);
}

void LcdRos::setCursor(int x, int y) { lcd.LCDSetCursor(x, y); }

void LcdRos::print(char const* c) { lcd.LCDPrintStr(c); }

void LcdRos::lcdCallback(std_msgs::StringConstPtr const& msg) {
  std::string str;
  str = msg->data;
  std::string segment;
  std::vector<std::string> contents;
  std::stringstream ss(str);
  while (getline(ss, segment, ':')) {
    contents.push_back(segment);
  }
  char char_array[contents[2].length() + 1];
  strcpy(char_array, contents[2].c_str());
  setCursor(std::stoi(contents[0]), std::stoi(contents[1]));
  print(char_array);
}

}  // namespace dira_lcd

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_publisher", ros::init_options::NoRosout);
  ros::NodeHandle nh;

  try {
    ROS_INFO("[LCD] Initializing node");
    dira_lcd::LcdRos lcd(nh);
    ros::spin();
  } catch (std::string const& ex) {
    ROS_FATAL_STREAM("[LCD] " << ex);
  }
  return 0;
}