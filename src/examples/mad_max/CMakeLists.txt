px4_add_module(
	MODULE examples__mad_max
	MAIN mad_max
	SRCS
		mad_max.cpp
		mad_max.hpp
	DEPENDS
		px4_work_queue
	)
