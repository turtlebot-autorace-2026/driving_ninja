#!/bin/bash

ninja_s

ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=false
