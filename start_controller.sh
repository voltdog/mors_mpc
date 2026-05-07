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

	echo "Launching Convex MPC Quadruped Controller..."

    SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
    echo "Script Location: $SCRIPT_DIR"

	config_dir="$SCRIPT_DIR/config/"
	echo "Config Location: ${config_dir}"

    # Fast DDS shared-memory transport often leaves stale locks in /dev/shm.
    # Force UDP transport for this local bringup to avoid SHM port conflicts.
    export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

    sim_mode=false
    rviz_enabled=false
    logging_enabled=false
    dcm_enabled=false

    for arg in "$@"; do
        case "$arg" in
            --sim)
                sim_mode=true
                ;;
            --rviz)
                rviz_enabled=true
                ;;
            --log)
                logging_enabled=true
                ;;
            --dcm)
                dcm_enabled=true
                ;;
            -h|--help)
                echo "Usage: $0 [--sim] [--rviz] [--log] [--dcm]" >&2
                exit 0
                ;;
            *)
                echo "Unknown option: $arg" >&2
                echo "Usage: $0 [--sim] [--rviz] [--log] [--dcm]" >&2
                exit 1
                ;;
        esac
    done

    # ros2 controller
    ros2 launch robot_mode_controller bringup.launch.py &
    pids+=($!)

    if [ "$sim_mode" = true ]; then
        echo "-------------- Simulation Mode Activated --------------" 
        ${SCRIPT_DIR}/.mpc_venv/bin/python ${SCRIPT_DIR}/Simulator/mors_simulator.py &
        pids+=($!)
        
        sleep 2s
    else
        # hardware interfaces
        echo "-------------- Hardware Mode Activated --------------"
        ${SCRIPT_DIR}/RealsenseCamera/build/realsense_camera  &
        pids+=($!)
        ${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu &
        pids+=($!)
        # in the future add here contact sensors controller
        # wait before the other controllers start to ensure that the hardware interfaces are up and running
        sleep 4s
    fi

    if [ "$rviz_enabled" = true ]; then
        if [ "$dcm_enabled" = true ]; then
            rviz_config="$SCRIPT_DIR/ros_ws/src/robot_state_viewer/rviz/rviz_config_dcm.rviz"
        else
            rviz_config="$SCRIPT_DIR/ros_ws/src/robot_state_viewer/rviz/rviz_config.rviz"
        fi
        ros2 launch robot_state_viewer robot_state_viewer.launch.py "rviz_config:=${rviz_config}" &
        pids+=($!)
        sleep 2s
    fi
    
    # state estimation
    # ${SCRIPT_DIR}/StateEstimator/build/state_estimator &
    # ${SCRIPT_DIR}/StateEstimatorLKF/build/state_estimator_lkf &

	# robot control
    if [ "$dcm_enabled" = true ]; then
        ${SCRIPT_DIR}/LocomotionControllerDCM/build/locomotionControllerDCM &
    else
        ${SCRIPT_DIR}/LocomotionController/build/locomotionControllerMPC & 
    fi
    pids+=($!)

    ${SCRIPT_DIR}/HeightMapBuilder/build/height_map_builder &
    pids+=($!)

    echo "Robot Controller Started Successfully"

	# data logger
    if [ "$logging_enabled" = true ]; then
	    ${SCRIPT_DIR}/MorsLogger/build/mors_logger &
        logger_pid=$!
        pids+=($logger_pid)
        (
            sleep 60s
            kill "$logger_pid" 2>/dev/null || true
            echo "[MorsLogger]: killed"
        ) &
    fi
	
	wait
)
