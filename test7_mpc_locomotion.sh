(trap 'kill 0' SIGINT; 
	echo "Launching Test 5: Body Control via Convex MPC..."

	config_dir="${PWD}/config/"
	echo "${config_dir}"
	# state estimation
	
	# simulation
	# ros2 launch mors_sim mors_pybullet.launch.py &
	
	# hardware
	./RealsenseCamera/build/realsense_camera  &
    ./BHI360_IMU/build/bhi360_imu &
    ./StateEstimator/build/state_estimator &
	
	sleep 3s

	# robot control
	# ./RobotModeController/build/robot_mode_controller & 
	
	# ./GaitScheduler/build/gait_scheduler &
	# python3 ./GaitScheduler/main.py --config "${config_dir}" &
	# python3 ./SwingController/main.py --config "${config_dir}" &
	# ./StanceControllerMPC/build/stanceControllerMPC  &
	./LocomotionController/build/locomotionControllerMPC &
	./LegController/build/leg_controller & 

	python3 ./examples/example5_mpc_locomotion.py &

	# data logger
	./MorsLogger/build/mors_logger &
	sleep 30s; kill $!; echo "MorsLogger killed" &

	wait
	)