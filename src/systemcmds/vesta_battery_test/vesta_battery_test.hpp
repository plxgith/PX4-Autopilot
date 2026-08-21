#pragma once

#include <stdio.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>


#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/test_motor.h>


#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


class VestaBatteryTest : public ModuleBase<VestaBatteryTest>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VestaBatteryTest();
	~VestaBatteryTest() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void motor_test(unsigned channel, float value, uint8_t driver_instance, int timeout_ms);
	void update_all_outputs(float value);
private:
	static constexpr double interval_us = 10000;	// 10ms; 100Hz

	void Run() override;

	hrt_abstime _last_time = 0;
	hrt_abstime _last_debug_time = 0;
	uint32_t counter = 0;
	uint8_t channel = 0;
	uint8_t motor = 0;
	// Battery Subscription
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	battery_status_s battery;
	float low_battery_v = 3.8;	// V

	uORB::Publication<debug_array_s> _debug_pub{ORB_ID(debug_array)};
	debug_array_s test_debug{0};


	uORB::Publication<test_motor_s> _motors_out_pub{ORB_ID(test_motor)};

	// phases of testing
	hrt_abstime time_soft_start = 5 * 1e6;
	hrt_abstime time_takeoff = 40 * 1e6;	// s
	hrt_abstime time_land = 40 * 1e6;


	// indexes for debug array
	uint8_t voltage = 2;
	uint8_t current = 3;
	uint8_t throttle = 4;
	uint8_t seconds_counter = 0;
	uint8_t phase_counter = 1;

	// phases throttle amounts
	float percent_soft_start = 0.1f;
	float percent_takeoff = 0.75f;
	float percent_cruise = 0.3f;
	float percent_land = 0.65f;
	float percent_done = -1;

	enum test_phase {
		soft_start,
		takeoff,
		cruise,
		land,
		finished
	}current_phase = soft_start;


	bool p_soft_start = false;
	bool p_takeoff = false;
	bool p_cruise = false;
	bool p_land = false;



};
