cmake_minimum_required(VERSION 3.18)
project(cds_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation sensor_msgs std_msgs
                                        geometry_msgs)

add_message_files(FILES sign.msg lane.msg circle_obstacle.msg
                  segment_obstacle.msg obstacles.msg)

add_service_files(FILES ResetLane.srv RecoverLane.srv IsTurnable.srv IncDecSpeed.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_generation sensor_msgs geometry_msgs
               std_msgs message_runtime)
