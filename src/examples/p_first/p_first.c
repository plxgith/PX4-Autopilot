/*
	@file p_first.c
	from "writing first application"

*/

// For PX4_INFO, PX4_ERROR, and other system messages
#include <px4_platform_common/log.h>

// sensor_combined - holds syncronizes sensor data of the complete sytem
#include <uORB/topics/sensor_combined.h>

// for polling and pollfd structs
#include <poll.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>




// Export main function so it can be accessed through-out the system
// Main function must be named <module_name>_main
__EXPORT int p_first_main(int argc, char *argv[]);

int p_first_main(int argc, char *argv[])
{
	//
	PX4_INFO("IN app");

	// app subscribes to sensor_combined topic
	// sensor_sub_fd is a topic handle

	int sensor_combined_fd = orb_subscribe(ORB_ID(sensor_combined));

	px4_pollfd_struct_t fds[] = {
		{.fd = sensor_combined_fd, .events=POLLIN },
	};

	while(true) {

		// wait for sensor update of 1 file descriptor for 1000ms
		int poll_ret = px4_poll(fds, 1, 1000);

		if(fds[0].revents & POLLIN) {
			// obtained data for the first file descriptor
			struct sensor_combined_s raw;
			// copy sensors raw data into local buffer
			orb_copy(ORB_ID(sensor_combined), sensor_combined_fd, &raw);

			PX4_INFO(" Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				(double)raw.accelerometer_m_s2[0],
				(double)raw.accelerometer_m_s2[1],
				(double)raw.accelerometer_m_s2[2]);
		}


	}

	return OK;
}
