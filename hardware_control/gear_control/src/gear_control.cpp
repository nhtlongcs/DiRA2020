#include "gear_control/gear_control.hpp"

#include <ros/param.h>
#include <map>
#include <thread>

namespace {
int normalize(double x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int constexpr THROTTLE_MAX_REVERSE = 410;
int constexpr THROTTLE_NEUTRAL = 614;
int constexpr THROTTLE_MAX_FORWARD = 819;

int constexpr STEERING_MAX_RIGHT = 475;
int constexpr STEERING_MAX_LEFT = 855;

// The THROTTLE is plugged into the following PWM channel
int constexpr THROTTLE_CHANNEL = 8;
// The Steering Servo is plugged into the following PWM channel
int constexpr STEERING_CHANNEL = 0;

int constexpr MIN_ANGLE = -35;
int constexpr MAX_ANGLE = 35;  // 10 * maxRight angle

int constexpr MIN_SERVO = 0;
int constexpr MAX_SERVO = 180;

}  // namespace

DiraGearNode::DiraGearNode(ros::NodeHandle &nh)
    : pca9685(std::make_unique<PCA9685>()) {
  std::string speedTopic, steerTopic, sensorTopic;
  ros::param::param<int>("~pwm_pca9685", pwmFreq_, 100);
  ros::param::param<std::string>("/set_speed_topic", speedTopic, "set_speed");
  ros::param::param<std::string>("/set_steer_topic", steerTopic, "set_angle");
  ros::param::param<std::string>("/laser_topic", sensorTopic, "ss2_status");

  onInit();
  ROS_INFO_STREAM("PCA9685 Device Address: 0x" << std::hex
                                               << pca9685->kI2CAddress);
  ROS_INFO_STREAM("PCA 9685 PWM frequency: " << pwmFreq_);

  pca9685->closePCA9685();

  speedSub_ =
      nh.subscribe(speedTopic, 5, &DiraGearNode::throttleCallback, this);
  steerSub_ = nh.subscribe(steerTopic, 5, &DiraGearNode::steerCallback, this);
  laserSub_ =
      nh.subscribe(sensorTopic, 5, &DiraGearNode::laserSensorCallback, this);
}

DiraGearNode::~DiraGearNode() {
  onInit();
  int pwm_steer_middle =
      normalize(0, MIN_ANGLE, MAX_ANGLE, STEERING_MAX_RIGHT, STEERING_MAX_LEFT);
  pca9685->setPWM(STEERING_CHANNEL, 0, pwm_steer_middle);
  pca9685->setPWM(THROTTLE_CHANNEL, 0, THROTTLE_NEUTRAL);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  pca9685->closePCA9685();
}

void DiraGearNode::onInit() {
  while (!pca9685->openPCA9685()) {
    pca9685->setAllPWM(0, 0);
    pca9685->reset();
    pca9685->setPWMFrequency(pwmFreq_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int pwm_steer_middle = normalize(0, MIN_ANGLE, MAX_ANGLE,
                                     STEERING_MAX_RIGHT, STEERING_MAX_LEFT);
    pca9685->setPWM(STEERING_CHANNEL, 0, pwm_steer_middle);
    pca9685->setPWM(THROTTLE_CHANNEL, 0, THROTTLE_NEUTRAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DiraGearNode::throttleCallback(std_msgs::Float32 const &throttle) {
  onInit();
  if (abs(throttle.data) < 60 && laserStatus_) {
    throttle_ = throttle.data;
    move();
  } else {
    brake();
    // throttle_ = 0;
  }
  pca9685->closePCA9685();
}

void DiraGearNode::laserSensorCallback(std_msgs::Bool const &sensor) {
  laserStatus_ = sensor.data;
  if (!laserStatus_) {
    onInit();
    brake();
    // throttle_ = 0;
    pca9685->closePCA9685();
  }
}

void DiraGearNode::steerCallback(std_msgs::Float32 const &steer) {
  onInit();
  double theta = steer.data;
  if (theta < MIN_ANGLE) {
    theta = MIN_ANGLE;
  }
  if (theta > MAX_ANGLE) {
    theta = MAX_ANGLE;
  }
  int pwm1 = normalize(theta, MIN_ANGLE, MAX_ANGLE, STEERING_MAX_RIGHT,
                       STEERING_MAX_LEFT);
  pca9685->setPWM(STEERING_CHANNEL, 0, pwm1);
  // pca9685->setPWM(STEERING_CHANNEL, 0, int(theta));
  pca9685->closePCA9685();
}

void DiraGearNode::move() {
  if (throttle_ >= 0) {
    if (direction_ == Direction::BACKWARD) {
      pca9685->setPWM(THROTTLE_CHANNEL, 0, THROTTLE_MAX_FORWARD);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      pca9685->setPWM(THROTTLE_CHANNEL, 0, THROTTLE_NEUTRAL);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    direction_ = Direction::FORWARD;
    int pwm =
        normalize(throttle_, 0, 100, THROTTLE_NEUTRAL, THROTTLE_MAX_FORWARD);
    pca9685->setPWM(THROTTLE_CHANNEL, 0, pwm);
    ROS_INFO_STREAM("Forward: " << throttle_ << "%, pwm: " << pwm);
  } else if (throttle_ < 0) {
    if (direction_ == Direction::FORWARD) {
      pca9685->setPWM(THROTTLE_CHANNEL, 0, THROTTLE_MAX_REVERSE);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      pca9685->setPWM(THROTTLE_CHANNEL, 0, THROTTLE_NEUTRAL);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    direction_ = Direction::FORWARD;
    int pwm =
        normalize(throttle_, 0, -100, THROTTLE_NEUTRAL, THROTTLE_MAX_REVERSE);
    pca9685->setPWM(THROTTLE_CHANNEL, 0, pwm);
    ROS_INFO_STREAM("Backward: " << throttle_ << "%, pwm: " << pwm);
  }
}

void DiraGearNode::brake() {
  if (direction_ == Direction::STOP) {
    pca9685->setPWM(THROTTLE_CHANNEL, 0, THROTTLE_NEUTRAL);
    ROS_INFO_STREAM("Braking but direction is stop");
  } else {
    if (abs(throttle_) > 9) {
      int pwm =
          (direction_ == Direction::FORWARD)
              ? normalize(-75, 0, -100, THROTTLE_NEUTRAL, THROTTLE_MAX_REVERSE)
              : normalize(75, 0, -100, THROTTLE_NEUTRAL, THROTTLE_MAX_FORWARD);
      pca9685->setPWM(THROTTLE_CHANNEL, 0, pwm);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ROS_INFO_STREAM("Hard braking");
    } else {
      ROS_INFO_STREAM("Soft braking");
    }
    pca9685->setPWM(THROTTLE_CHANNEL, 0, THROTTLE_NEUTRAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  direction_ = Direction::STOP;
  throttle_ = 0;
}
