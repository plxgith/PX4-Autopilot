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
	// run every 10ms
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

	// if new battery data
	if(_battery_sub.updated()) {
		battery_status_s battery;

		if(_battery_sub.copy(&battery)){

			// Adjust throttle based on current

		}



	}
	hrt_abstime test_time = hrt_absolute_time();

	if(test_time - _last_time > 1e6) {
		PX4_INFO("Now");
		_last_time = test_time;
		counter++;

		for(int i = 0; i < 10; i++) {
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


