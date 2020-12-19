# DiRa Gear Controller

## Prerequisites

1. Calibrate the ESC [guide](https://www.jetsonhacks.com/2016/01/26/jetson-racecar-part-3-esc-motor-control/), remember to edit the **throttle** , **channel** and **frequency** value
2. Verify that the car is using **Sport** mode
3. Build this package with great passion
4. `roslaunch gear_control gear_debug.launch`

## Default parameters

- `speed_topic`	"set_speed"
- `steer_topic`	"set_angle"
- `pwm_pca9685`	"100"

## How it works

- Run when -100 < `speed_topic` < 100, otherwise brake