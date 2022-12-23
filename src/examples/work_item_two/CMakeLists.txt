px4_add_module(
	MODULE examples__work_item_two
	MAIN work_item_example_two
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		WorkItemExampleTwo.cpp
		WorkItemExampleTwo.hpp
	DEPENDS
		px4_work_queue
	)
