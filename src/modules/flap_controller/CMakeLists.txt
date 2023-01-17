px4_add_module(
	MODULE modules__flap_controller
	MAIN flap_controller
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		flap_controller.cpp
		flap_controller.hpp
	DEPENDS
		px4_work_queue
	)
