#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run harvesting border_move" &
#bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run sllidar_ros2 sllidar_node" &
#bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run lidar get_main_obstacles" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel twist_to_rpm" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel data_exchange"



#xterm -hold -e "./start_binocular.sh" &
#xterm -hold -e "./start_view_raws.sh"
