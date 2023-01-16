
# px4_add_module builds static library from a module description

px4_add_module(
	MODULE examples__p_first
	MAIN p_first
	COMPILE_FLAGS
	SRCS
		hello_main.c
	DEPENDS
	)
