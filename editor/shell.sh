#!/bin/bash

# ros2 run my_package my_node --ros-args -p my_param:=my_value
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null
# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash >/dev/null
ros2 run csv_editor csv_editor_node --ros-args -p base_path:="$1" >/dev/null
