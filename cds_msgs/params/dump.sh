#!/bin/bash

rosrun dynamic_reconfigure dynparam dump /planning planning.yaml
rosrun dynamic_reconfigure dynparam dump /carcontrol carcontrol.yaml
rosrun dynamic_reconfigure dynparam dump /detectobject detectobject.yaml
rosrun dynamic_reconfigure dynparam dump /lanedetect lanedetect.yaml
rosrun dynamic_reconfigure dynparam dump /sign_detect sign_detect.yaml