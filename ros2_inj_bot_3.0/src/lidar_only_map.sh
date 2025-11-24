#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
export DISPLAY=:0
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run lidar view_median_map"
