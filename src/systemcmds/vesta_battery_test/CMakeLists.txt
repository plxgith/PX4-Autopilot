px4_add_module(
	MODULE systemcmds__vesta_battery_test
	MAIN vesta_battery_test
	STACK_MAIN 4096
	SRCS
		vesta_battery_test.cpp
	)

