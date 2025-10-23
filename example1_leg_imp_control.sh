(trap 'kill 0' SIGINT; 
    echo "Launching Test 1: Leg Control..."
    config_dir="${PWD}/config/"
	echo "${config_dir}"
	# state estimation
	
	# simulation
	# ros2 launch mors_sim mors_pybullet.launch.py &
	
	# hardware
	./RealsenseCamera/build/realsense_camera  &
    ./BHI360_IMU/build/bhi360_imu &
    sleep 4s
    ./StateEstimator/build/state_estimator &


	./LegController/build/leg_controller & 
	# ./RobotModeController/build/robot_mode_controller & 
	# ./MorsLogger/build/mors_logger &
	python3 ./examples/example1_leg.py 
	wait
	# fg
	) 