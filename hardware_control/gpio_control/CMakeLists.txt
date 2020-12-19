cmake_minimum_required(VERSION 3.18)
project(gpio_control)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_python_setup()

catkin_package(
  # INCLUDE_DIRS include LIBRARIES dira_gpio_controller CATKIN_DEPENDS rospy
  # std_msgs DEPENDS system_lib
)

include_directories(# include
                    ${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/gpio_control.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
