/* Independent module running on it's own with a task to control
*  elevon position during VTOL forward transition
*
* Position of elevons are a function of airspeed
*/

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
//#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

// Since VTOL_attitude_control_main publishes it
#include <uORB/topics/actuator_controls.h>
// Need airspeed topic
#include <uORB/topics/airspeed.h>
// Need to know in which state the VTOL is
#include <uORB/topics/vtol_vehicle_status.h>
// To listen in which mode it is(auto, mission, etc)
//#include <uORB/topics/vehicle_control_mode.h>
// To send data to QGround?
#include <uORB/topics/debug_value.h>

// To send Mavlink Commands
#include <uORB/topics/vehicle_command.h>

// To see Mavlink Messages
//#include <v2.0/standard/standard.h>
//#include <MAVLink/mavlink_messages.h>
#include <uORB/topics/actuator_outputs.h>




using namespace time_literals;

class FlapController : public ModuleBase<FlapController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FlapController();
	~FlapController() override;

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
	//uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	// Publishes calculated elevon angle
	// actuator_controls_1 for VTOL
	uORB::Publication<actuator_controls_s>	_actuator_controls_pub{ORB_ID(actuator_controls_1)};
	uORB::Publication<debug_value_s>	_debug_value_pub{ORB_ID(debug_value)};

	// send mavlink command inside system to control pitch during transition
	uORB::Publication<vehicle_command_s>	_vehicle_command{ORB_ID(vehicle_command)};

	// direct actuator control
	uORB::Publication<actuator_outputs_s>	_actuator_out_pub{ORB_ID(actuator_outputs)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	// Subscribe to vehicle status topic to get system_id and other
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data



	// Subscribes to airspeed needed to calculate the flap output
	uORB::SubscriptionCallbackWorkItem _airspeed_sub{this, ORB_ID(airspeed)};
	// Subscribe to topic and listen if the vehicle is in transition mode
	uORB::Subscription		   _vehicle_vtol_state_sub{ORB_ID(vtol_vehicle_status)};

	// Subscribe to VTOL actuator topic to see values
	uORB::Subscription		   _actuator_controls_1_sub{ORB_ID(actuator_controls_1)};



	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Variables
	actuator_controls_s 	_flap_command{};
	vehicle_command_s	_mav_vehicle_command{};
	vehicle_status_s	_vehicle_status{};

	// Debug Variables
	debug_value_s		_debug_value{};
	actuator_controls_s	_debug_actuator_commands{};


	bool _armed{false};
};

