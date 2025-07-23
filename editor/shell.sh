#!/bin/bash

# ros2 run my_package my_node --ros-args -p my_param:=my_value
source /opt/ros/humble/setup.bash >/dev/null
source /aichallenge/workspace/install/setup.bash >/dev/null
# ros2 run csv_editor csv_editor_node --ros-args -p base_path:=$1 >/dev/null
echo "Setting CSV path to $1"
ros2 param set /planning/scenario_planning/simple_trajectory_generator csv_path $1 >/dev/null
