#include "vesta_battery_test.hpp"


VestaBatteryTest::VestaBatteryTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	PX4_INFO("Constructor done");
}

int VestaBatteryTest::task_spawn(int argc, char *argv[])
{
	VestaBatteryTest *instance = new VestaBatteryTest();

	if(instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;


		// if hardware scheduled work item ok
		if(instance->init()) {
			PX4_INFO("Object instance created");
			return PX4_OK;
		}
		else {
			PX4_ERR("alloc failed");
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;

		return PX4_ERROR;
	}
}

bool VestaBatteryTest::init()
{

	_last_time = hrt_absolute_time();
   	 _last_debug_time = _last_time;
	test_debug.data[voltage] = 777;
	ScheduleOnInterval(interval_us);
	PX4_INFO("Module Scheduled");
	return true;
}


int VestaBatteryTest::custom_command(int argc, char *argv[])
{
	return 0;
}

int VestaBatteryTest::print_usage(const char *reason)
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

void VestaBatteryTest::Run()
{
	PX4_INFO("Vesta Battery Test Run Loop");

	if(should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		PX4_INFO("Module closed");
		return;
	}

	// get current time
	hrt_abstime test_time = hrt_absolute_time();


	// SLOW START PHASE
	if(test_time - _last_time > time_soft_start && current_phase == soft_start) {
		test_debug.data[voltage] = 888;
		_last_time = test_time;	// remember time
		current_phase = takeoff;
		PX4_INFO("SOFT_START -> TAKEOFF");
	}

	// TAKEOFF PHASE
	if(test_time - _last_time > time_takeoff && current_phase == takeoff) {
	test_debug.data[voltage] = 999;
	_last_time = test_time;	// remember time
	current_phase = cruise;
    	PX4_INFO("TAKEOFF -> CRUISE");
	}




	// if new battery data
	if(_battery_sub.updated()) {
		battery_status_s battery;

		if(_battery_sub.copy(&battery)){

			// test - copy battery voltage
			test_debug.data[5] = battery.voltage_v;
			// test - copy battery current
			test_debug.data[6] = battery.current_a;
			// test - adjust throt

			// Adjust throttle based on current


		}



	}

	if(test_time - _last_debug_time > 1e6) {
		PX4_INFO("Now");
		_last_debug_time = test_time;
		counter++;

		for(int i = 1; i < 5; i++) {
			test_debug.data[i] = counter;
		}
		// out.motor_number = motor_number;
		// out.value = channel;
		// out.action = counter;

	}
	_motors_out_pub.publish(out);
	_debug_pub.publish(test_debug);



}




extern "C" __EXPORT int vesta_battery_test_main(int argc, char *argv[])
{
	return VestaBatteryTest::main(argc, argv);
}


