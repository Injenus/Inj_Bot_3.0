#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run servo one_thread_send_receive" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel twist_to_rpm_omni" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel data_exchange" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run pharma_delivery init_pos"