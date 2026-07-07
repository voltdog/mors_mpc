#!/bin/bash

set -euo pipefail

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

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"

rviz_enabled=false
map_enabled=false

for arg in "$@"; do
    case "$arg" in
        --rviz)
            rviz_enabled=true
            ;;
        --map)
            map_enabled=true
            ;;
        -h|--help)
            echo "Usage: $0 [--rviz] [--map]" >&2
            exit 0
            ;;
        *)
            echo "Unknown option: $arg" >&2
            echo "Usage: $0 [--rviz] [--map]" >&2
            exit 1
            ;;
    esac
done

# Fast DDS shared-memory transport often leaves stale locks in /dev/shm.
# Force UDP transport for this local bringup to avoid SHM port conflicts.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

echo "Launching state estimation stack..."
echo "Script Location: $SCRIPT_DIR"

# # RealSense T265 odometry.
# "${SCRIPT_DIR}/RealsenseCamera/build/realsense_camera" &
# pids+=($!)

# # RealSense D435i depth image publisher. HeightMapBuilder consumes DEPTH_IMAGE
# # and can turn it into POINTCLOUD/HEIGHTMAP when that module is running.
# "${SCRIPT_DIR}/RealsenseCameraD435i/build/realsense_camera_d435i" &
# pids+=($!)

"${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu" &
pids+=($!)

"${SCRIPT_DIR}/StateEstimatorHMB/build/state_estimator_hmb" &
pids+=($!)

# # Wait before starting the estimator so hardware publishers have time to come up.
# sleep 4s

# "${SCRIPT_DIR}/StateEstimator/build/state_estimator" &
# pids+=($!)

# if [ "$map_enabled" = true ]; then
#     "${SCRIPT_DIR}/HeightMapBuilder/build/height_map_builder" &
#     pids+=($!)
# fi

if [ "$rviz_enabled" = true ]; then
    rviz_config="$SCRIPT_DIR/ros_ws/src/robot_state_viewer/rviz/rviz_config.rviz"
    ros2 launch robot_state_viewer robot_state_viewer.launch.py "rviz_config:=${rviz_config}" &
    pids+=($!)
    sleep 2s
fi

echo "State estimation stack started successfully"

wait
