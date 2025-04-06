#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
bash -c "source /home/inj/Inj_Bot_3.0/venv/bin/activate && source /home/inj/Inj_Bot_3.0/ros2_inj_bot_3.0/install/setup.bash && ros2 bag play /home/inj/Inj_Bot_3.0/ros2_inj_bot_3.0/logger_bag/omni_pharm_3" &
bash -c "source /home/inj/Inj_Bot_3.0/venv/bin/activate && source /home/inj/Inj_Bot_3.0/ros2_inj_bot_3.0/install/setup.bash && ros2 run fuck_wheel data_exchange"