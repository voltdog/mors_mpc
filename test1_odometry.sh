(trap 'kill 0' SIGINT; 
	ros2 launch point_lio mapping_unilidar_l1.launch.py &
	sleep 5s
	ros2 launch unitree_lidar_ros2 launch.py &
	
	./MorsLogger/build/mors_logger &
	sleep 30s; kill $!; echo "MorsLogger killed" &
	wait
	)