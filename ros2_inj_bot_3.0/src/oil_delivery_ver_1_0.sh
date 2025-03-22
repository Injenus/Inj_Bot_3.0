#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel data_exchange" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel twist_to_rpm" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run servo one_thread_send_receive" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run camera arm" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run camera binocular" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run oil_delivery ver_1_0"
#bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run keyboard detect_key"

