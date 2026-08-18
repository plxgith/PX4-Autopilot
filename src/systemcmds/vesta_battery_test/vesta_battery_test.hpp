#include <stdio.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>


#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/debug_array.h>

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

private:
	static constexpr double interval_us = 10000;	// 10ms; 100Hz

	void Run() override;

	time_t _last_time;
	// Battery Subscription
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};


};
