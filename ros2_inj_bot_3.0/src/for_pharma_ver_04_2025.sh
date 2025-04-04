#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run camera arm" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run camera recog_aruco" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run camera recog_qr_code" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel twist_to_rpm_omni" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel data_exchange" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run sllidar_ros2 sllidar_node" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run lidar get_main_obstacles" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run servo one_thread_send_receive" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run servo lut_control"
