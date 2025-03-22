#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

xterm -hold -e "./arm_start_n_saving.sh" &
xterm -hold -e "./binocular_start_n_saving.sh"

