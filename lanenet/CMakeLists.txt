cmake_minimum_required(VERSION 2.8.3)
project(lanenet)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        message_runtime
        message_generation
        cv_bridge
)

## Generate messages in the 'msg' folder
 add_message_files(
         FILES
         Curve.msg
         Lane.msg
         Lane_Image.msg
 )

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
   CATKIN_DEPENDS roscpp rospy message_runtime std_msgs sensor_msgs
)