(trap 'kill 0' SIGINT; 
	./RealsenseCamera/build/realsense_camera  &
	./BHI360_IMU/build/bhi360_imu &
	sleep 3s
	
	./MorsLogger/build/mors_logger &
	sleep 30s; kill $!; echo "MorsLogger killed" &
	wait
	)