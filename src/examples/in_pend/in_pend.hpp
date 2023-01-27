#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp> 	// for work item
#include <px4_platform_common/posix.h>

// uORB stuff
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

// uORB Topics
#include <uORB/topics/vehicle_angular_velocity.h>			// For gyro data
#include <uORB/topics/vehicle_acceleration.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_actuator_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

// timers
#include <drivers/drv_hrt.h>

class inPend : public ModuleBase<inPend>, public ModuleParams, public px4::WorkItem
{

public:
	inPend();
	~inPend() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();




private:
	void Run() override;



	/* Subscriptions */
	//uORB::Subscription				_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	//uORB::Subscription				_v_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription				_v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};

	uORB::SubscriptionCallbackWorkItem		_vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionCallbackWorkItem		_vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};


	/* Publications */
	//uORB::Publication<actuator_controls_s>		_actuators_0_pub;
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};





};
