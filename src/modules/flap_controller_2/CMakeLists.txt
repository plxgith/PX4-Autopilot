px4_add_module(
	MODULE modules__flap_controller_2
	MAIN flap_controller_2
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		flap_controller_2.cpp
		flap_controller_2.hpp
	DEPENDS
		px4_work_queue

	)
