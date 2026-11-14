#!/usr/bin/env bash

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    printf '사용법: source "%s"\n' "$0" >&2
    exit 1
fi

cd "$HOME/turtlebot3_ws" || return
source .venv/bin/activate || return
source /opt/ros/galactic/setup.bash || return
colcon build --symlink-install || return
source install/setup.bash
