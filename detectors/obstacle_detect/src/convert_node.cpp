#include <geometry_msgs/Point.h>
#include <cds_msgs/circle_obstacle.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>

ros::Publisher pub;

void obstacleCallback(const cds_msgs::circle_obstacle& msg)
{
    std_msgs::Int8 outMsg;
    if (msg.center.x < 0)
    {
        outMsg.data = -1;
    } else if (msg.center.x > 0)
    {
        outMsg.data = 1;
    } else
    {
        ROS_WARN("object x = 0");
    }
    pub.publish(outMsg);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_obstacle_lidar");
    ros::NodeHandle nh;
    std::string obstacle_topic, obstacle_direct_topic;


    ROS_ASSERT(ros::param::get("/obstacles", obstacle_topic));
    ROS_ASSERT(ros::param::get("/object", obstacle_direct_topic));

    ros::Subscriber sub = nh.subscribe(obstacle_topic, 1, obstacleCallback);
    pub = nh.advertise<std_msgs::Int8>(obstacle_direct_topic, 1);
    ros::spin();

    return 0;
}