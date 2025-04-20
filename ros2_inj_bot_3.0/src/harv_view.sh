#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

bash -c "\
    source ../../venv/bin/activate && \
    source ../install/setup.bash && \
    export PYTHONPATH=\"\$PYTHONPATH:/home/inj/Inj_Bot_3.0/venv/lib/python3.11/site-packages\" && \
    ros2 run harvesting obj_detection"