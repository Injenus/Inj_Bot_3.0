#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
export DISPLAY=:0
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run sllidar_ros2 sllidar_node" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run lidar get_main_obstacles" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run lidar view_median_map"
