(trap 'kill 0' SIGINT; 
	./RealsenseCamera/build/realsense_camera  &
    ./BHI360_IMU/build/bhi360_imu &
    sleep 4s
    ./StateEstimator/build/state_estimator &
	# sleep 3s
	
	# ./MorsLogger/build/mors_logger &
	# sleep 15s; kill $!; echo "MorsLogger killed" &
	wait
	)