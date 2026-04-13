/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file motor_test.cpp
 *
 * Tool for drive testing (extended, smooth sine, safe timeout)
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
	bool was_running = false;

	const int update_hz = 5000; // 5 kHz update rate for smooth sine
	const int update_us = 1000000 / update_hz;

	while (true) {

		if (!g_running) {
			// stop motors if we just transitioned from running
			if (was_running) {
				if (g_channel < 0) {
					for (int i = 0; i < 8; ++i) motor_test(i, -1.f, g_driver, 0);
				} else {
					motor_test(g_channel, -1.f, g_driver, 0);
				}
				PX4_INFO("motors stopped");
				start = 0;
			}
			was_running = false;
			px4_usleep(10000);
			continue;
		}

		was_running = true;

		if (start == 0) start = hrt_absolute_time();
		hrt_abstime now = hrt_absolute_time();

		// timeout
		if (g_timeout_ms > 0 && (now - start) >= (hrt_abstime)g_timeout_ms * 1000) {
			g_running = false;
			g_mode = TestMode::NONE;
			continue; // next loop will stop motors
		}

		float val = 0.f;

		switch (g_mode) {

		case TestMode::STATIC:
			val = g_value;
			break;

		case TestMode::MIN_MAX: {
			double t = (now - start) / 1e6; // seconds
			double period = 1.0 / g_freq;
			double phase = fmod(t, period) / period;
			// linear ramp up/down for smoother square wave
			if (phase < 0.5) {
				val = g_min + (g_max - g_min) * (phase * 2.0);
			} else {
				val = g_max - (g_max - g_min) * ((phase - 0.5) * 2.0);
			}
			break;
		}

		case TestMode::SINE: {
			double t = (now - start) / 1e6; // seconds

			// hold minimum value for first 3 seconds
			if (t < 3.0) {
				val = g_min;
			} else {
				double t_shifted = t - 3.0; // start sine after delay
				double amp = (g_max - g_min) * 0.5;
				double offset = g_min + amp;
				val = offset + amp * sin(2.0 * M_PI * g_freq * t_shifted);
			}
			break;
		}

		case TestMode::NONE:
		default:
			val = -1.f;
			break;
		}

		// apply value to selected channels
		if (g_channel < 0) {
			for (int i = 0; i < 8; ++i) motor_test(i, val, g_driver, 0);
		} else {
			motor_test(g_channel, val, g_driver, 0);
		}

		// high-resolution sleep
		hrt_abstime elapsed = hrt_absolute_time() - now;
		int dt = update_us - (int)elapsed;
		if (dt > 0) px4_usleep(dt);
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* START WORKER                                                       */
/* ------------------------------------------------------------------ */

static void start_worker()
{
	if (g_task >= 0) return;
	g_mode = TestMode::NONE;
	px4_usleep(10000);


	g_task = px4_task_spawn_cmd("motor_test_worker",
				    SCHED_DEFAULT,
				    SCHED_PRIORITY_DEFAULT,
				    3000,
				    motor_test_worker,
				    nullptr);
}

/* ------------------------------------------------------------------ */
/* USAGE                                                              */
/* ------------------------------------------------------------------ */

static void usage(const char *reason)
{
	if (reason) PX4_WARN("%s", reason);

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Motor test utility (smooth sine, safe timeout).

WARNING: remove all props before using.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("motor_test", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Static motor value");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_min_max", "Toggle between min/max with ramp");
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

	if (strcmp(cmd, "stop") == 0) {
		g_running = false;
		g_mode = TestMode::NONE;
		for (int i = 0; i < 8; ++i) motor_test(i, -1.f, driver, 0);
		return 0;
	}

	start_worker();

	if (strcmp(cmd, "test") == 0) {
		g_channel = channel;
		g_value = value;
		g_driver = driver;
		g_timeout_ms = timeout_ms;
		g_mode = TestMode::STATIC;
		g_running = true;
		return 0;
	}

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
