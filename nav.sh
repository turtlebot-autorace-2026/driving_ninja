#!/bin/bash

ninja_s

ros2 launch turtlebot3_navigation2 navigation2.launch.py map:='/home/atom5/map.yaml' use_sim_time:=False
