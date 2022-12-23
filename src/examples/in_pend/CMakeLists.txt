px4_add_module(
	MODULE examples__in_pend
	MAIN in_pend
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	STACK_MAX
		1300
	SRCS
		in_pend.cpp
		in_pend.hpp
	DEPENDS

		RateControl
		px4_work_queue
	)
