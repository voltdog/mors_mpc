(trap 'kill 0' SIGINT; 
	# state
	./RealsenseCamera/build/realsense_camera  &
    ./BHI360_IMU/build/bhi360_imu &
    ./StateEstimator/build/state_estimator &

	# leg control
	./LegController/build/leg_controller & 
	# run example
	python3 ./examples/example6_contact_estimation.py --leg 2 & 

	# data logger
	./MorsLogger/build/mors_logger &
	sleep 15s; kill $!; echo "MorsLogger killed" &

	wait)