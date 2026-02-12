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


	# после запуска test3_state_estimation надо запустить пакет для картирования
	# ros2 launch mors_mapping mapping_lcm.launch.py 

	# Если хочется визуализировать всё, то надо запустить конвертер lcm->ROS для карты высот
	# ros2 launch mors_mapping lcm_grid_map_visualizer.launch.py 

	# Запуск rviz
	# rviz2
	# конфиг для него можно взять в ~/mors_experiments/ros_ws/src/mors_mapping/utils/mapping.rviz