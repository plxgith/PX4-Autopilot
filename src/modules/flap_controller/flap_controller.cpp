#include "flap_controller.hpp"

FlapController::FlapController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

FlapController::~FlapController()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool FlapController::init()
{
	// execute Run() on every sensor_accel publication
	/*
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	*/
	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate


	// execute Run() on every airspeed publication
	if(!_airspeed_sub.registerCallback()) {
		PX4_ERR("airspeed callback registration failed");
		return false;
	}





	return true;
}

void FlapController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}



	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}

	// comment out, it's unnecessary, but keep for reference
	/*
	//  grab latest accelerometer data
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// DO WORK
			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}
	*/


	// grab latest airspeed data
	if(_airspeed_sub.updated()) {
		airspeed_s p_airspeed;	// struct for data

		// Publish to debug just to check it's getting there
		//_debug_value.value = 69;
		//_debug_value_pub.publish(_debug_value);



		// Try getting pitch elevon pitch angle to publish to debug
		_actuator_controls_1_sub.copy(&_debug_actuator_commands); // copy into struct
		/* struct actuator_controls_s {
		*	uint64_t timestamp
		*	uint64_t timestamp_sample
		*	float control[8]
		*/
		}
		_debug_value.value = _debug_actuator_commands.control[actuator_controls_s::INDEX_PITCH];
		_debug_value_pub.publish(_debug_value);
		PX4_INFO("%f", double(_debug_actuator_commands.control))


		// copy to buffer
		if(_airspeed_sub.copy(&p_airspeed)) {
			// print for debug
			//PX4_INFO("%f", double(p_airspeed.indicated_airspeed_m_s));

			vtol_vehicle_status_s VTOL_state; // struct for VTOL state
			if(_vehicle_vtol_state_sub.copy(&VTOL_state)) {
				// if it's in transition
				if(VTOL_state.in_transition_to_fw) {
					PX4_INFO("In transition");

					/*
					 * struct actuator_control_s *get_actuators_fw_in()
					 */
					_flap_command.control[actuator_controls_s::INDEX_PITCH] = 45.0f;	// actuator_controls[1][1] -> pitch

					// publish to uORB
					// but maybe need to copy other commander before
					// (other axis)
					_actuator_controls_pub.publish(_flap_command);
					_debug_value.value = _flap_command.control[actuator_controls_s::INDEX_PITCH];
					// publish to topic
					_debug_value_pub.publish(_debug_value);

				}
			}
		}




	}




	perf_end(_loop_perf);
}

int FlapController::task_spawn(int argc, char *argv[])
{
	FlapController *instance = new FlapController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FlapController::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int FlapController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlapController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Flap Controller module running on a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flap_controller", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flap_controller_main(int argc, char *argv[])
{
	return FlapController::main(argc, argv);
}
