/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file motor_test.c
 *
 * Tool for drive testing (extended)
 */

#include <drivers/drv_hrt.h>
#include <math.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/test_motor.h>

extern "C" __EXPORT int motor_test_main(int argc, char *argv[]);

/* ------------------------------------------------------------------ */
/* STATE                                                              */
/* ------------------------------------------------------------------ */

enum class TestMode {
	NONE = 0,
	STATIC,
	MIN_MAX,
	SINE
};

static volatile TestMode g_mode = TestMode::NONE;
static volatile bool g_running = false;

static int g_channel = -1;
static float g_value = 0.f;
static float g_min = 0.f;
static float g_max = 0.f;
static float g_freq = 1.f;
static int g_timeout_ms = 0;
static uint8_t g_driver = 0;

static int g_task = -1;

/* ------------------------------------------------------------------ */
/* BASIC MOTOR COMMAND                                                */
/* ------------------------------------------------------------------ */

static void motor_test(unsigned channel, float value, uint8_t driver_instance, int timeout_ms)
{
	test_motor_s test_motor{};
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = channel;
	test_motor.value = value;
	test_motor.action = value >= 0.f ? test_motor_s::ACTION_RUN : test_motor_s::ACTION_STOP;
	test_motor.driver_instance = driver_instance;
	test_motor.timeout_ms = timeout_ms;

	uORB::Publication<test_motor_s> pub{ORB_ID(test_motor)};
	pub.publish(test_motor);
}

/* ------------------------------------------------------------------ */
/* WORKER                                                             */
/* ------------------------------------------------------------------ */

static int motor_test_worker(int argc, char *argv[])
{
	hrt_abstime start = 0;

	while (true) {

		if (!g_running) {
			px4_usleep(10000);
			continue;
		}

		if (start == 0) {
			start = hrt_absolute_time();
		}

		hrt_abstime now = hrt_absolute_time();

		/* timeout */
		if (g_timeout_ms > 0 &&
		    (now - start) >= (hrt_abstime)g_timeout_ms * 1000) {
			g_running = false;
			g_mode = TestMode::NONE;
		}

		switch (g_mode) {

		case TestMode::STATIC:

			if (g_channel < 0) {
				for (int i = 0; i < 8; ++i) {
					motor_test(i, g_value, g_driver, 0);
				}
			} else {
				motor_test(g_channel, g_value, g_driver, 0);
			}

			px4_usleep(10000);
			break;

		case TestMode::MIN_MAX: {

			static bool high = false;
			const int half_period_us = (int)(1e6f / g_freq / 2.f);
			float val = high ? g_max : g_min;

			if (g_channel < 0) {
				for (int i = 0; i < 8; ++i) {
					motor_test(i, val, g_driver, 0);
				}
			} else {
				motor_test(g_channel, val, g_driver, 0);
			}

			high = !high;
			px4_usleep(half_period_us);
			break;
		}

		case TestMode::SINE: {

			float t = (now - start) / 1e6f;
			float amp = (g_max - g_min) * 0.5f;
			float offset = g_min + amp;
			float val = offset + amp * sinf(2.f * M_PI_F * g_freq * t);

			if (g_channel < 0) {
				for (int i = 0; i < 8; ++i) {
					motor_test(i, val, g_driver, 0);
				}
			} else {
				motor_test(g_channel, val, g_driver, 0);
			}

			px4_usleep(1000);
			break;
		}

		case TestMode::NONE:
		default:
			start = 0;
			px4_usleep(10000);
			break;
		}
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* START WORKER                                                       */
/* ------------------------------------------------------------------ */

static void start_worker()
{
	if (g_task >= 0) {
		return;
	}

	g_task = px4_task_spawn_cmd("motor_test_worker",
				    SCHED_DEFAULT,
				    SCHED_PRIORITY_DEFAULT,
				    2000,
				    motor_test_worker,
				    nullptr);
}

/* ------------------------------------------------------------------ */
/* USAGE                                                              */
/* ------------------------------------------------------------------ */

static void usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Motor test utility (extended).

WARNING: remove all props before using.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("motor_test", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Static motor value");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_min_max", "Toggle between min/max");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_sine", "Sine wave test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop all motors");

	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 8, "Motor", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 100, "Power", true);
	PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, 100, "Min", true);
	PRINT_MODULE_USAGE_PARAM_INT('h', 0, 0, 100, "Max", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('f', 1.0f, 0.1f, 100.0f, "Freq Hz", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 100, "Timeout s", true);
}

/* ------------------------------------------------------------------ */
/* MAIN                                                               */
/* ------------------------------------------------------------------ */

int motor_test_main(int argc, char *argv[])
{
	int channel = -1;
	float value = 0.f;
	float min_value = 0.f;
	float max_value = 0.f;
	float frequency = 1.f;
	int timeout_ms = 0;
	uint8_t driver = 0;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:m:p:t:l:h:f:", &myoptind, &myoptarg)) != EOF) {

		switch (ch) {

		case 'i': driver = atoi(myoptarg); break;
		case 'm': channel = atoi(myoptarg) - 1; break;
		case 'p': value = atof(myoptarg) / 100.f; break;
		case 'l': min_value = atof(myoptarg) / 100.f; break;
		case 'h': max_value = atof(myoptarg) / 100.f; break;
		case 'f': frequency = atof(myoptarg); break;
		case 't': timeout_ms = atoi(myoptarg) * 1000; break;

		default:
			usage(nullptr);
			return 1;
		}
	}

	if (myoptind >= argc) {
		usage(nullptr);
		return 1;
	}

	const char *cmd = argv[myoptind];

	/* STOP */
	if (strcmp(cmd, "stop") == 0) {

		g_running = false;
		g_mode = TestMode::NONE;

		for (int i = 0; i < 8; ++i) {
			motor_test(i, -1.f, driver, 0);
		}

		return 0;
	}

	start_worker();

	/* TEST */
	if (strcmp(cmd, "test") == 0) {

		g_channel = channel;
		g_value = value;
		g_driver = driver;
		g_timeout_ms = timeout_ms;

		g_mode = TestMode::STATIC;
		g_running = true;
		return 0;
	}

	/* MIN MAX */
	if (strcmp(cmd, "test_min_max") == 0) {

		if (max_value <= min_value) {
			PX4_WARN("max must be > min");
			return 1;
		}

		g_channel = channel;
		g_min = min_value;
		g_max = max_value;
		g_freq = frequency;
		g_driver = driver;
		g_timeout_ms = timeout_ms;

		g_mode = TestMode::MIN_MAX;
		g_running = true;
		return 0;
	}

	/* SINE */
	if (strcmp(cmd, "test_sine") == 0) {

		if (max_value <= min_value) {
			PX4_WARN("max must be > min");
			return 1;
		}

		g_channel = channel;
		g_min = min_value;
		g_max = max_value;
		g_freq = frequency;
		g_driver = driver;
		g_timeout_ms = timeout_ms;

		g_mode = TestMode::SINE;
		g_running = true;
		return 0;
	}

	usage(nullptr);
	return 1;
}
