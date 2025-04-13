#!/bin/bash
SCENARIO=${1:-"default"}  # Получаем первый аргумент или используем "default"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

#amixer set Master 65535

if [ "$SCENARIO" = "omni" ]; then
    bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel twist_to_rpm_omni" &
else
    bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel twist_to_rpm" &
fi

bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run fuck_wheel data_exchange" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run mems uiiaiuiiai"

#amixer set Master 16384

