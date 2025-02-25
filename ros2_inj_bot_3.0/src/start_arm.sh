#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
bash -c "source ../../venvbin/activate && source ../install/setup.bash && ros2 run camera arm"