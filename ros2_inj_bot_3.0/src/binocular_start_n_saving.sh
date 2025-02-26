#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

xterm -hold -e "./start_binocular.sh" &
xterm -hold -e "./save_v_binocular.sh" 