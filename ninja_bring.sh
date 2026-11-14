#!/usr/bin/env bash

sudo chmod 777 /dev/ttyACM0 &&
    sudo chmod 777 /dev/ttyUSB0 &&
    ros2 launch turtlebot3_bringup robot.launch.py
