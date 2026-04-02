/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Holger Steinhaus <hsteinhaus@gmx.de>
 *
 ****************************************************************************/

/**
 * @file motor_test.c
 *
 * Tool for drive testing
 */

#include <drivers/drv_hrt.h>
#include <math.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/test_motor.h>

extern "C" __EXPORT int motor_test_main(int argc, char *argv[]);

static void motor_test(unsigned channel, float value, uint8_t driver_instance, int timeout_ms);
static void motor_test_min_max(int channel, float min_val, float max_val,
			      uint8_t driver_instance, int timeout_ms, float frequency_hz);
static void motor_test_sine(int channel, float min_val, float max_val,
			   uint8_t driver_instance, int timeout_ms, float frequency_hz);
static void usage(const char *reason);

void motor_test(unsigned channel, float value, uint8_t driver_instance, int timeout_ms)
{
	test_motor_s test_motor{};
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = channel;
	test_motor.value = value;
	test_motor.action = value >= 0.f ? test_motor_s::ACTION_RUN : test_motor_s::ACTION_STOP;
	test_motor.driver_instance = driver_instance;
	test_motor.timeout_ms = timeout_ms;

	uORB::Publication<test_motor_s> test_motor_pub{ORB_ID(test_motor)};
	test_motor_pub.publish(test_motor);

	if (test_motor.action == test_motor_s::ACTION_STOP) {
		PX4_INFO("motors stop command sent");
	} else {
		PX4_INFO("motor %d set to %.2f", channel + 1, (double)value);
	}
}

static void motor_test_min_max(int channel, float min_val, float max_val,
			      uint8_t driver_instance, int timeout_ms, float frequency_hz)
{
	const int period_us = (int)(1e6f / frequency_hz);
	const int half_period_us = period_us / 2;

	hrt_abstime start = hrt_absolute_time();

	while (true) {

		if (timeout_ms > 0) {
			hrt_abstime now = hrt_absolute_time();
			if ((now - start) >= (hrt_abstime)timeout_ms * 1000) {
				break;
			}
		}

		// HIGH
		if (channel < 0) {
			for (int i = 0; i < 8; ++i) {
				motor_test(i, max_val, driver_instance, 0);
			}
		} else {
			motor_test(channel, max_val, driver_instance, 0);
		}
		px4_usleep(half_period_us);

		// LOW
		if (channel < 0) {
			for (int i = 0; i < 8; ++i) {
				motor_test(i, min_val, driver_instance, 0);
			}
		} else {
			motor_test(channel, min_val, driver_instance, 0);
		}
		px4_usleep(half_period_us);
	}

	// Stop motors
	if (channel < 0) {
		for (int i = 0; i < 8; ++i) {
			motor_test(i, -1.f, driver_instance, 0);
		}
	} else {
		motor_test(channel, -1.f, driver_instance, 0);
	}

	PX4_INFO("test_min_max finished");
}

static void motor_test_sine(int channel, float min_val, float max_val,
			   uint8_t driver_instance, int timeout_ms, float frequency_hz)
{
	const float amplitude = (max_val - min_val) * 0.5f;
	const float offset = min_val + amplitude;

	const int step_us = 1000; // 1 ms loop
	hrt_abstime start = hrt_absolute_time();

	while (true) {

		hrt_abstime now = hrt_absolute_time();
		float t = (now - start) / 1e6f;

		if (timeout_ms > 0) {
			if ((now - start) >= (hrt_abstime)timeout_ms * 1000) {
				break;
			}
		}

		float value = offset + amplitude * sinf(2.f * M_PI_F * frequency_hz * t);

		if (channel < 0) {
			for (int i = 0; i < 8; ++i) {
				motor_test(i, value, driver_instance, 0);
			}
		} else {
			motor_test(channel, value, driver_instance, 0);
		}

		px4_usleep(step_us);
	}

	// Stop motors
	if (channel < 0) {
		for (int i = 0; i < 8; ++i) {
			motor_test(i, -1.f, driver_instance, 0);
		}
	} else {
		motor_test(channel, -1.f, driver_instance, 0);
	}

	PX4_INFO("test_sine finished");
}

static void usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to test motors.

WARNING: remove all props before using this command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("motor_test", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Set motor(s) to a specific output value");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 8, "Motor to test (1...8, all if not specified)", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 100, "Power (0...100)", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 100, "Timeout in seconds", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 4, "driver instance", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("test_min_max", "Alternate between min and max power");
	PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, 100, "Min power (0...100)", true);
	PRINT_MODULE_USAGE_PARAM_INT('h', 0, 0, 100, "Max power (0...100)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('f', 1.0f, 0.1f, 100.0f, "Frequency in Hz", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("test_sine", "Run motors with sine wave between min and max");

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop all motors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate", "Iterate all motors");
}

int motor_test_main(int argc, char *argv[])
{
	int channel = -1;
	unsigned long lval;
	float value = 0.0f;
	float min_value = 0.0f;
	float max_value = 0.0f;
	float frequency = 1.0f;

	uint8_t driver_instance = 0;
	int ch;
	int timeout_ms = 0;

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:m:p:t:l:h:f:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'i':
			driver_instance = (uint8_t)strtol(myoptarg, nullptr, 0);
			break;

		case 'm':
			channel = (int)strtol(myoptarg, nullptr, 0) - 1;
			break;

		case 'p':
			lval = strtoul(myoptarg, nullptr, 0);
			if (lval > 100) {
				usage("value invalid");
				return 1;
			}
			value = ((float)lval) / 100.f;
			break;

		case 't':
			timeout_ms = strtol(myoptarg, nullptr, 0) * 1000;
			break;

		case 'l':
			lval = strtoul(myoptarg, nullptr, 0);
			if (lval > 100) {
				usage("min invalid");
				return 1;
			}
			min_value = ((float)lval) / 100.f;
			break;

		case 'h':
			lval = strtoul(myoptarg, nullptr, 0);
			if (lval > 100) {
				usage("max invalid");
				return 1;
			}
			max_value = ((float)lval) / 100.f;
			break;

		case 'f':
			frequency = strtof(myoptarg, nullptr);
			if (frequency <= 0.f) {
				usage("frequency invalid");
				return 1;
			}
			break;

		default:
			usage(nullptr);
			return 1;
		}
	}

	bool run_test = true;

	if (myoptind >= 0 && myoptind < argc) {

		if (strcmp("stop", argv[myoptind]) == 0) {
			channel = 0;
			value = -1.f;

		} else if (strcmp("iterate", argv[myoptind]) == 0) {
			value = 0.15f;

			for (int i = 0; i < 8; ++i) {
				motor_test(i, value, driver_instance, 0);
				px4_usleep(500000);
				motor_test(i, -1.f, driver_instance, 0);
				px4_usleep(10000);
			}

			run_test = false;

		} else if (strcmp("test_min_max", argv[myoptind]) == 0) {
			run_test = false;

			if (max_value <= min_value) {
				PX4_WARN("max must be greater than min");
				return 1;
			}

			motor_test_min_max(channel, min_value, max_value,
					   driver_instance, timeout_ms, frequency);

		} else if (strcmp("test_sine", argv[myoptind]) == 0) {
			run_test = false;

			if (max_value <= min_value) {
				PX4_WARN("max must be greater than min");
				return 1;
			}

			motor_test_sine(channel, min_value, max_value,
					driver_instance, timeout_ms, frequency);

		} else if (strcmp("test", argv[myoptind]) == 0) {
			// default

		} else {
			usage(nullptr);
			return 0;
		}

	} else {
		usage(nullptr);
		return 0;
	}

	if (run_test) {
		if (channel < 0) {
			for (int i = 0; i < 8; ++i) {
				motor_test(i, value, driver_instance, timeout_ms);
				px4_usleep(10000);
			}
		} else {
			motor_test(channel, value, driver_instance, timeout_ms);
		}
	}

	return 0;
}
