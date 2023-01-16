/*
	@file p_first.c
	from "writing first application"

*/

// For PX4_INFO, PX4_ERROR, and other system messages
#include <px4_platform_common/log.h>

// sensor_combined - holds syncronizes sensor data of the complete sytem
//#include <uORB/topics/sensor_combined.h>


// Export main function so it can be accessed through-out the system
// Main function must be named <module_name>_main
extern "C" __EXPORT int p_first_main(int argc, char *argv[]);

int p_first_main(int argc, char argv*[])
{
	//
	PX4_INFO("IN app");

	// app subscribes to sensor_combined topic
	// sensor_sub_fd is a topic handle
	/*
	int sensor_combined_fd = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[] = {
		.fd = sensor_combined_fd, .events=POLLIN,
	};

	while(true){


	}
	*/
	return OK;
}
