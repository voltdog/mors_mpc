#!/bin/bash

set -euo pipefail

(

    pids=()

    cleanup() {
        trap - SIGINT SIGTERM EXIT
        if [ ${#pids[@]} -gt 0 ]; then
            kill "${pids[@]}" 2>/dev/null || true
        fi
        pkill -P $$ 2>/dev/null || true
        wait 2>/dev/null || true
    }

    trap cleanup SIGINT SIGTERM EXIT

    echo "Launching Heightmap Builder..."

    SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
    echo "Script Location: $SCRIPT_DIR"

    "${SCRIPT_DIR}/RealsenseCameraD435i/build/realsense_camera_d435i" &
    pids+=($!)

    # sleep 2s

    # ${SCRIPT_DIR}/HeightMapBuilder/build/height_map_builder &
    # pids+=($!)


    echo "Heightmap Builder stack started successfully"

    wait

)