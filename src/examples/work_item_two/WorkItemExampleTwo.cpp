#include "WorkItemExampleTwo.hpp"

WorkItemExampleTwo::WorkItemExampleTwo() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::test2)
{
}

WorkItemExampleTwo::~WorkItemExampleTwo()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExampleTwo::init()
{
	// Uncomment for the module on which update period you want to run the controller
	/*
	// execute Run() on every sensor_accel publication
	if (!_sensor_accel_sub.registerCallback()) {
		PX4_ERR("sensor_accel callback registration failed");
		return false;
	}
	*/
	/*
	if (!_sensor_baro_sub.registerCallback()) {
		PX4_ERR("sensor_baro callback registration failed");
		return false;
	}
	*/

	if (!_sensor_gyro_sub.registerCallback()) {
		PX4_ERR("sensor_gyro callback registration failed");
		return false;
	}
	// alternatively, Run on fixed interval
	 //ScheduleOnInterval(10000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void WorkItemExampleTwo::Run()
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


	// Example
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


	// Example
	//  grab latest accelerometer data
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s accel;

		if (_sensor_accel_sub.copy(&accel)) {
			// DO WORK
			PX4_INFO("Pitch Rate Parameter=%f\n", (double)_param_pend_p.get());

			// access parameter value (SYS_AUTOSTART)
			if (_param_sys_autostart.get() == 1234) {
				// do something if SYS_AUTOSTART is 1234
			}
		}
	}


	// Example
	//  publish some data
	/*
	orb_test_s data{};
	data.val = 314159;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data);
	*/


	perf_end(_loop_perf);
}

int WorkItemExampleTwo::task_spawn(int argc, char *argv[])
{
	WorkItemExampleTwo *instance = new WorkItemExampleTwo();

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

int WorkItemExampleTwo::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExampleTwo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExampleTwo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int work_item_example_two_main(int argc, char *argv[])
{
	return WorkItemExampleTwo::main(argc, argv);
}
