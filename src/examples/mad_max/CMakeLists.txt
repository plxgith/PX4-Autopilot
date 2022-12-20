px4_add_module(
	MODULE examples__mad_max
	MAIN mad_max
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		MadMax.cpp
		MadMax.hpp
	DEPENDS
		px4_work_queue
	)
