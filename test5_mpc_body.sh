(trap 'kill 0' SIGINT; 
	echo "Launching Test 5: Body Control via Convex MPC..."
	# state estimation
	
	# simulation
	# ros2 launch mors_sim mors_pybullet.launch.py &
	
	# hardware
	./RealsenseCamera/build/realsense_camera  &
    ./BHI360_IMU/build/bhi360_imu &
    ./StateEstimator/build/state_estimator &
	sleep 3s

	# robot control
	./LegController/build/leg_controller & 
	./StanceControllerMPC/build/stanceControllerMPC  & 
	# ./RobotModeController/build/robot_mode_controller & 

	python3 ./examples/example4_body_mpc.py &

	# data logger
	./MorsLogger/build/mors_logger &
	sleep 10s; kill $!; echo "MorsLogger killed" &

	wait
	)