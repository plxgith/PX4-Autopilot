

/**
 *
 *
 *  File with various output test functions
 *
 *  Try to test out torque setpoints, various outputs, etc
 *
 */


#pragma once
// Start / Stop module
 #include <px4_platform_common/module.h>
 #include <px4_platform_common/module_params.h>

 // For PX4_ISFINITE checks
 #include <px4_platform_common/defines.h>

 // Work Queue
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB Stuff

// Publish/Subscribe
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

// Topics
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/log_message.h>
// Debug Value Topic and Debug Array to watch in QGroundControl
#include <uORB/topics/debug_value>
#include <uORB/topics/debug_array.h>




// Should be started as a module running on it's own queue?
 class OutputTest: public ModuleBase<OutputTest>, public px4::ScheduledWorkItem
 {

public:
	// Constructor
	OutputTest();
	~OutputTest() override;

	// Module Functions
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;


	// Additional Functions
	int test_torque(float amount);


private:

	void Run() override;

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem
	uORB::Subscription

	// Publications
	uORB::Publication


	// Parameters
	DEFINE_PARAMETERS (
		// Example Parameters
		(ParamInt<px4::params::SYS_AUTOSTART) 	_param_sys_autostart,
		(ParamInt<px4::params::SYS_AUTOCONFIG) 	_param_sys_autoconfig

		// My Parameters

	)

	bool _armed{false};

 }
