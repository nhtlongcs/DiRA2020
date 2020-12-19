#include <std_msgs/Bool.h>
#include <string>
#include <ros/ros.h>

class Button {
 public:
  typedef void (*Callback)(bool);

  Button(Callback cb, const std::string& name = "")
      : _name{name}, _isPressed{false}, _callback{cb} {}

  bool isPressed() const { return _isPressed; }

  void callback(const std_msgs::Bool& msg) {
    if (_isPressed != msg.data) {
      _isPressed = msg.data;
      if (_isPressed) {
        ROS_DEBUG_STREAM("Button " << _name << " Pressed");
      } else {
        ROS_DEBUG_STREAM("Button " << _name << " Released");
      }
      _callback(_isPressed);
    }
  }

 private:
  std::string _name;
  bool _isPressed;
  Callback _callback;
};
