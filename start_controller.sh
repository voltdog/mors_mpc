#!/bin/bash

(trap 'kill 0' SIGINT; 
	echo "Launching Convex MPC Quadruped Controller..."

    SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
    echo "Script Location: $SCRIPT_DIR"

	config_dir="$SCRIPT_DIR/config/"
	echo "Config Location: ${config_dir}"

    # ros2 controller
    ros2 launch robot_mode_controller bringup.launch.py &

    # Проверяем аргумент
    if [ "$1" == "--sim" ]; then
        echo "-------------- Simulation Mode Activated --------------" 
        /home/yoggi/mors_mpc/.mpc_venv/bin/python ${SCRIPT_DIR}/Simulator/mors_simulator.py &
        sleep 4s
    else
        # hardware interfaces
        echo "-------------- Hardware Mode Activated --------------"
        ${SCRIPT_DIR}/RealsenseCamera/build/realsense_camera  &
        ${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu &
        # in the future add here contact sensors controller
        # wait before the other controllers start to ensure that the hardware interfaces are up and running
        sleep 4s
    fi

    
    # state estimation
    # ${SCRIPT_DIR}/StateEstimator/build/state_estimator &
    ${SCRIPT_DIR}/StateEstimatorLKF/build/state_estimator_lkf &

	# robot control
	${SCRIPT_DIR}/LocomotionController/build/locomotionControllerMPC &
	${SCRIPT_DIR}/LegController/build/leg_controller & 

    echo "Robot Controller Started Successfully" &

	# data logger
    if [ "$2" == "--log" ]; then
	    ${SCRIPT_DIR}/MorsLogger/build/mors_logger &
        sleep 30s; kill $!; echo "[MorsLogger]: killed" &
    fi
	
	wait
	)

# 3.118
# 3.110
# 5.723
# 3.134
# 3.185
# 2.844
# 3.165
# 3.142
# 8.022
# 3.182
# 3.080
# 0.377