#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

// Add
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>

//#include "params.h"

using namespace time_literals;

class WorkItemExampleTwo : public ModuleBase<WorkItemExampleTwo>, public ModuleParams, public px4::WorkItem
{
public:
	WorkItemExampleTwo();
	~WorkItemExampleTwo() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Publications


	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionCallbackWorkItem _sensor_baro_sub {this, ORB_ID(sensor_baro)};
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub {this, ORB_ID(sensor_gyro)};

	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) 		_param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) 	_param_sys_autoconfig,  /**< another parameter */

		// use multicopter Pitch Rate slider for Pendulum Tuning
		(ParamFloat<px4::params::MC_PITCHRATE_P>)	_param_pend_p
	)


	bool _armed{false};
};
