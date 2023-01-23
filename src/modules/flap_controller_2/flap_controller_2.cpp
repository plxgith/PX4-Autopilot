#include "flap_controller_2.hpp"

FlapController2::FlapController2() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

FlapController2::~FlapController2()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool FlapController2::init()
{
	// execute Run() on every sensor_accel publication

	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate


	// execute Run() on every airspeed publication
	if(!_airspeed_sub.registerCallback()) {
		PX4_ERR("airspeed callback registration failed");
		return false;
	}







	return true;
}

void FlapController2::Run()
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
	// Log data into Debug for ULog analysis;
	// Needs to run for a JMavSim Multirotor, so outside of checking
	// Airspeed Message

	//  grab latest accelerometer data
	/*
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// DO WORK
			//_debug_value.value = 667.23;
			//_debug_value_pub.publish(_debug_value);

			_debug_value.value = _debug_actuator_commands.control[actuator_controls_s::INDEX_PITCH];
			_debug_value_pub.publish(_debug_value);
			}
	}
	*/



	// grab latest airspeed data
	if(_airspeed_sub.updated()) {
		//airspeed_s p_airspeed;	// struct for data

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

		// Published data to show in QGroundControl
		_debug_value.value = _debug_actuator_commands.control[actuator_controls_s::INDEX_PITCH];
		_debug_value_pub.publish(_debug_value);

		// print to console
		//PX4_INFO("%f", double(_debug_value.value));

		// Publishes data to get written in the log
		/*
		_debug_array.data[0] = _debug_value.value;
		_debug_array.timestamp = _debug_actuator_commands.timestamp;
		_debug_array_pub.publish(_debug_array);

		*/


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
					// scale according to airspeed
					_flap_command.control[actuator_controls_s::INDEX_PITCH] = p_airspeed.indicated_airspeed_m_s * kP_airspeed;	// actuator_controls[1][1] -> pitch
					PX4_INFO("%f", double(_flap_command.control[actuator_controls_s::INDEX_PITCH]));

					// publish to uORB
					// but maybe need to copy other commander before
					// (other axis)
					_actuator_controls_pub.publish(_flap_command);
					_debug_value.value = _flap_command.control[actuator_controls_s::INDEX_PITCH];
					_debug_array.data[0] = _debug_value.value;
					_debug_array.timestamp = _debug_actuator_commands.timestamp;
					_debug_array_pub.publish(_debug_array);
					// publish to topic
					//_debug_value_pub.publish(_debug_value);

				}
			}
		}




	}




	perf_end(_loop_perf);
}

int FlapController2::task_spawn(int argc, char *argv[])
{
	FlapController2 *instance = new FlapController2();

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

int FlapController2::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int FlapController2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlapController2::print_usage(const char *reason)
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

extern "C" __EXPORT int flap_controller_2_main(int argc, char *argv[])
{
	return FlapController2::main(argc, argv);
}
