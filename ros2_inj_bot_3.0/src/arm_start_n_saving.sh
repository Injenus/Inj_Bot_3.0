#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

xterm -hold -e "./start_arm.sh" &
xterm -hold -e "./save_v_arm.sh" 