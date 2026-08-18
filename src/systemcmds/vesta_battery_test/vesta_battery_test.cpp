#include "vesta_battery_test.hpp"


VestaBatteryTest::VestaBatteryTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

int VestaBatteryTest::task_spawn(int argc, char *argv[])
{
	VestaBatteryTest *instance = new VestaBatteryTest();

	if(instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;


		// if hardware scheduled work item ok
		if(instance->init()) {
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
	PX4_INFO("Vesta Battery Test Run Start");
	if(should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// if new battery data
	if(_battery_sub.updated()) {
		battery_status_s battery;

		if(_battery_sub.copy(&battery)){

			// Adjust throttle based on current

		}



	}
	time_t test_time = hrt_absolute_time();

	if(test_time - _last_time > 1e6) {
		PX4_INFO("Now");
	}


}



extern "C" __EXPORT int vesta_battery_test_main(int argc, char *argv[])
{
	return VestaBatteryTest::main(argc, argv);
}


