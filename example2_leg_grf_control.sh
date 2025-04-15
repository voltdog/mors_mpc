(trap 'kill 0' SIGINT; 
	./LegController/build/leg_controller & 
	./RobotModeController/build/robot_mode_controller & 
	./MorsLogger/build/mors_logger &
	python3 ./examples/example2_leg.py 
	wait
	# fg
	)