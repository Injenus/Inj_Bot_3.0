#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run camera arm" &
bash -c "source ../../venv/bin/activate && source ../install/setup.bash && ros2 run camera view_raws"

#xterm -hold -e "./start_binocular.sh" &
#xterm -hold -e "./start_view_raws.sh"
