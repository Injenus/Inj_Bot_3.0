#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run sllidar_ros2 sllidar_node" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run lidar get_main_obstacles" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run servo one_thread_send_receive" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel data_exchange" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel twist_to_rpm" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run harvesting border_move" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run harvesting debug_keyboard" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run harvesting move_to_box_short_side" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run harvesting start_fihish" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run harvesting arm_actions" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run servo lut_control"
#bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run harvesting obj_detection"
# bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run harvesting vers_04_25"
