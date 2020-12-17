#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include "cds_msgs/IncDecSpeed.h"
#include "hw_control/hw_control.hpp"

ros::Publisher lcdPub;
ros::ServiceClient maxSpeedSrv;
ros::ServiceClient minSpeedSrv;

static bool isSetForMaxVelocity = false;

void button1Callback(bool isPressed)
{
    static bool isMapRed = false;
    std_msgs::String msg;
    int constexpr x = 1;
    int constexpr y = 1;
    if (isPressed)
    {
        isMapRed = !isMapRed;
        std::ostringstream ss;
        ss << x << ':' << y << ':' << "Map " << (isMapRed ? "RED" : "BLUE");
        msg.data = ss.str();
        lcdPub.publish(msg);
        ROS_INFO("%s", ss.str().c_str());

        //TODO: Trigger planning service
    }
}

void button2Callback(bool isPressed)
{
    std_msgs::String msg;
    int constexpr x = 0;
    int y = 0;
    if (isPressed)
    {
        isSetForMaxVelocity = !isSetForMaxVelocity;
        std::ostringstream ss;
        if (isSetForMaxVelocity)
        {
            y = 1;
        }
        ss << x << ':' << 1-y << ':' << " ";
        ss << x << ':' << y << ':' << "*";
        ROS_INFO("%s", ss.str().c_str());
        msg.data = ss.str();
        lcdPub.publish(msg);
    }
}


void button3Callback(bool isPressed)
{
    int constexpr x = 5;
    int y = 0;
    if (isPressed)
    {
        std::ostringstream ss;
        // ss << x << ':' << y << ':' << (isSetForMaxVelocity ? "MAX" : "MIN") << " velocity = ";

        if(isSetForMaxVelocity) y = 1;
        ss << x << ":" << y << ":";

        cds_msgs::IncDecSpeed srv;
        srv.request.mode = srv.request.MODE_INC;

        if (isSetForMaxVelocity)
        {
            if (maxSpeedSrv.call(srv))
            {
                
                ss << std::to_string(srv.response.currentSpeed);
                std_msgs::String msg;
                msg.data = ss.str();
                lcdPub.publish(msg);
                ROS_INFO("%s", ss.str().c_str());
            }
            else
            {
                ROS_ERROR("Failed to call service for max speed");
            }
        }
        else
        {
            if (minSpeedSrv.call(srv))
            {
                ss << std::to_string(srv.response.currentSpeed);
                std_msgs::String msg;
                msg.data = ss.str();
                lcdPub.publish(msg);
                ROS_INFO("%s", ss.str().c_str());
            }
            else
            {
                ROS_ERROR("Failed to call service for min speed");
            }    
        }
    }
}


void button4Callback(bool isPressed)
{
    int constexpr x = 5;
    int y = 0;
    if (isPressed)
    {
        std::ostringstream ss;
        // ss << x << ':' << y << ':' << (isSetForMaxVelocity ? "MAX" : "MIN") << " velocity = ";

        if(isSetForMaxVelocity) y = 1;
        ss << x << ":" << y << ":";

        cds_msgs::IncDecSpeed srv;
        srv.request.mode = srv.request.MODE_DEC;

        if (isSetForMaxVelocity)
        {
            if (maxSpeedSrv.call(srv))
            {
                ss << std::to_string(srv.response.currentSpeed);
                std_msgs::String msg;
                msg.data = ss.str();
                lcdPub.publish(msg);
                ROS_INFO("%s", ss.str().c_str());
            }
            else
            {
                ROS_ERROR("Failed to call service for max speed");
            }
        }
        else
        {
            if (minSpeedSrv.call(srv))
            {
                ss << std::to_string(srv.response.currentSpeed);
                std_msgs::String msg;
                msg.data = ss.str();
                lcdPub.publish(msg);
                ROS_INFO("%s", ss.str().c_str());
            }
            else
            {
                ROS_ERROR("Failed to call service for min speed");
            }    
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hw_control_node");
    ros::NodeHandle nh;

    // Button class only call the callback once
    Button button1(&button1Callback, "1");
    Button button2(&button2Callback, "2");
    Button button3(&button3Callback, "3");
    Button button4(&button4Callback, "4");
    Laser laser;

    std::string btn1Topic, btn2Topic, btn3Topic, btn4Topic, laserTopic, lcdTopic;
    ROS_ASSERT(ros::param::get("button1_topic", btn1Topic));
    ROS_ASSERT(ros::param::get("button2_topic", btn2Topic));
    ROS_ASSERT(ros::param::get("button3_topic", btn3Topic));
    ROS_ASSERT(ros::param::get("button4_topic", btn4Topic));
    ROS_ASSERT(ros::param::get("laser_topic", laserTopic));
    ROS_ASSERT(ros::param::get("lcd_topic", lcdTopic));
    
    ros::Subscriber btn1Sub = nh.subscribe(btn1Topic, 10, &Button::callback, &button1);
    ros::Subscriber btn2Sub = nh.subscribe(btn2Topic, 10, &Button::callback, &button2);
    ros::Subscriber btn3Sub = nh.subscribe(btn3Topic, 10, &Button::callback, &button3);
    ros::Subscriber btn4Sub = nh.subscribe(btn4Topic, 10, &Button::callback, &button4);
    ros::Subscriber laserSub = nh.subscribe(laserTopic, 10, &Laser::callback, &laser);
    lcdPub = nh.advertise<std_msgs::String>(lcdTopic, 10);

    std::string min_speed_srv, max_speed_srv;
    ROS_ASSERT(ros::param::get("inc_dec_min_vel_srv", min_speed_srv));
    ROS_ASSERT(ros::param::get("inc_dec_max_vel_srv", max_speed_srv));

    minSpeedSrv = nh.serviceClient<cds_msgs::IncDecSpeed>(min_speed_srv);
    maxSpeedSrv = nh.serviceClient<cds_msgs::IncDecSpeed>(max_speed_srv);

    ROS_INFO("HW_Control Node started!");
    std_msgs::String msg;
    msg.data = "1:0:MIN";
    lcdPub.publish(msg);
    msg.data = "1:1:MAX";
    lcdPub.publish(msg);
    ros::spin();
    return 0;
}