#include <ros/ros.h>
#include "hw_control/hw_control.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hw_control_node");
    ros::NodeHandle nh;

    Button button1, button2, button3, button4;
    Laser laser;
    nh.subscribe("/button1_topic", 1, &Button::callback, &button1);
    nh.subscribe("/button2_topic", 1, &Button::callback, &button2);
    nh.subscribe("/button3_topic", 1, &Button::callback, &button3);
    nh.subscribe("/button4_topic", 1, &Button::callback, &button4);
    nh.subscribe("/laser_topic", 1, &Laser::callback, &laser);

    ros::spin();
    return 0;
}