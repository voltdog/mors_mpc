(trap 'kill 0' SIGINT; 
	echo "Launching Test 5: Body Control via Convex MPC..."

    SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
    echo "$SCRIPT_DIR"

	config_dir="$SCRIPT_DIR/config/"
	echo "${config_dir}"

    # ros2 controller
    ros2 launch robot_mode_controller bringup.launch.py &
	
	# hardware interfaces
	${SCRIPT_DIR}/RealsenseCamera/build/realsense_camera  &
    ${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu &
    sleep 4s
    ${SCRIPT_DIR}/StateEstimator/build/state_estimator &

	# robot control
	${SCRIPT_DIR}/LocomotionController/build/locomotionControllerMPC &
	${SCRIPT_DIR}/LegController/build/leg_controller & 

	# python3 ./examples/example5_mpc_locomotion.py &

	# data logger
	${SCRIPT_DIR}/MorsLogger/build/mors_logger &
	# sleep 30s; kill $!; echo "MorsLogger killed" &

    # echo "Robot Controller Started Successfully" &

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