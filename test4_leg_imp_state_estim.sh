(trap 'kill 0' SIGINT; 
	echo "Lunching Test 4: Leg Impedance Control + State Estimation + DataLogger ..."
	# state estimation
	./RealsenseCamera/build/realsense_camera  &
    ./BHI360_IMU/build/bhi360_imu &
    ./StateEstimator/build/state_estimator &
	sleep 3s
	# leg control
	./LegController/build/leg_controller & 
	./RobotModeController/build/robot_mode_controller & 
	python3 ./examples/example1_leg.py &
	sleep 1s
	# data logger
	./MorsLogger/build/mors_logger &
	sleep 20s; kill $!; echo "MorsLogger killed" &
	wait
	)