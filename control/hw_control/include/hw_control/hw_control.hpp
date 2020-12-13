#include <std_msgs/Bool.h>
#include <string>
#include <ros/ros.h>


class Button
{
public:
    Button(): _name{""}, _isPressed{false} {}

    bool isPressed() const {
        return _isPressed;
    }

    void callback(const std_msgs::Bool& msg)
    {
        if (_isPressed != msg.data)
        {
            _isPressed = msg.data;
            if (_isPressed)
            {
                ROS_DEBUG_STREAM("Button " << _name << " Pressed");
            }
            else
            {
                ROS_DEBUG_STREAM("Button " << _name << " Released");
            }
        }
    }

private:
    std::string _name;
    bool _isPressed;
};


class Laser
{
public:
    Laser(): _name{""} {}

    bool isDetected() const
    {
        return _isDetected;
    }

    void callback(const std_msgs::Bool& msg)
    {
        if (_isDetected != msg.data)
        {
            _isDetected = msg.data;
            if (_isDetected)
            {
                ROS_DEBUG_STREAM("Laser " << _name << " detected");
            }
            else
            {
                ROS_DEBUG_STREAM("Laser " << _name << " reset");
            }
        }
    }
private:
    std::string _name;
    bool _isDetected;
};