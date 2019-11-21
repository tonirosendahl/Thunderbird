
px4_add_board(
	PLATFORM nuttx
	VENDOR yuneec
	MODEL typhoon_h
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common

	SERIAL_PORTS

	DRIVERS
		adc
		barometer/ms5611
		#distance_sensor
		gps
		imu/mpu6000
		magnetometer/hmc5883
		magnetometer/ist8310
		px4fmu
		rc_input
		tap_esc
		#telemetry # all available telemetry drivers
		test_ppm
		#typhoon_bind
		st16_telemetry

	MODULES
		attitude_estimator_q
		commander
		dataman
		ekf2
		events
		land_detector
		#landing_target_estimator
		load_mon
		#local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		sensors
		sih
		vmount
		battery_status

	SYSTEMCMDS
		dmesg
		config
		hardfault_log
		#led_control
		mixer
		motor_test
		nshterm
		param
		perf
		pwm
		reflect
		#tests # tests and test runner
		top
		topic_listener
		tune_control
		#usb_connected
		ver

	EXAMPLES
		#bottle_drop # OBC challenge
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hwtest # Hardware test
		#matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#segway
		#uuv_example_app

	)
