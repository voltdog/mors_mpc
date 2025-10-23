(trap 'kill 0' SIGINT; 
	echo "Launching Test 5: Body Control via Convex MPC..."

    SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
    echo "$SCRIPT_DIR"

	config_dir="$SCRIPT_DIR/config/"
	echo "${config_dir}"
	# state estimation
	
	# simulation
	# ros2 launch mors_sim mors_pybullet.launch.py &
	
	# hardware
	${SCRIPT_DIR}/RealsenseCamera/build/realsense_camera  &
    ${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu &
    ${SCRIPT_DIR}/StateEstimator/build/state_estimator &
	
	sleep 4s

	# robot control
	# ./RobotModeController/build/robot_mode_controller & 
	
	# ./GaitScheduler/build/gait_scheduler &
	# python3 ./GaitScheduler/main.py --config "${config_dir}" &
	# python3 ./SwingController/main.py --config "${config_dir}" &
	# ./StanceControllerMPC/build/stanceControllerMPC  &
	${SCRIPT_DIR}/LocomotionController/build/locomotionControllerMPC &
	${SCRIPT_DIR}/LegController/build/leg_controller & 

	# python3 ./examples/example5_mpc_locomotion.py &

	# data logger
	${SCRIPT_DIR}/MorsLogger/build/mors_logger &
    # ./MorsLogger/build/mors_logger &
	# sleep 30s; kill $!; echo "MorsLogger killed" &

    # echo "Robot Controller Started Successfully" &

	wait
	)