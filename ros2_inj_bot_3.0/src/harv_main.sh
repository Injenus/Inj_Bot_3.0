#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

bash -c "\
    source ../../venv/bin/activate && \
    source ../install/setup.bash && \
    ros2 run harvesting vers_04_25"