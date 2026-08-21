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
	test_debug.data[phase_counter] = percent_soft_start;
	test_debug.id = 1;
	update_all_outputs(percent_soft_start);	// this has to be divided???
	ScheduleOnInterval(interval_us);
	PX4_INFO("Module Scheduled");
	return true;
}

void VestaBatteryTest::motor_test(unsigned channel, float value, uint8_t driver_instance, int timeout_ms)
{
	test_motor_s test_motor{};
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = channel;
	test_motor.value = value;
	test_motor.action = value >= 0.f ? test_motor_s::ACTION_RUN : test_motor_s::ACTION_STOP;
	test_motor.driver_instance = driver_instance;
	test_motor.timeout_ms = timeout_ms;

	uORB::Publication<test_motor_s> pub{ORB_ID(test_motor)};
	pub.publish(test_motor);
}

void VestaBatteryTest::update_all_outputs(float value)
{
	for (int i = 0; i < 7; ++i) {
		 motor_test(i, value, 0, 0);
		 px4_usleep(1000);
	}

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
Battery Testing module. Mimics battery discharge on Vesta VTOL

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vesta_battery_test", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void VestaBatteryTest::Run()
{
	//PX4_INFO("Vesta Battery Test Run Loop");

	if(should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		PX4_INFO("Module closed");
		update_all_outputs(-1);
		return;
	}

	// get current time
	hrt_abstime test_time = hrt_absolute_time();


	// SLOW START PHASE
	if(test_time - _last_time > time_soft_start && current_phase == soft_start) {
		test_debug.data[phase_counter] = percent_takeoff;
		test_debug.id = 2;
		update_all_outputs(percent_takeoff);	// update
		_last_time = test_time;	// remember time
		current_phase = takeoff;
	}

	// TAKEOFF PHASE
	if(test_time - _last_time > time_takeoff && current_phase == takeoff) {
	test_debug.data[phase_counter] = percent_cruise;
	update_all_outputs(percent_cruise);
	test_debug.id = 3;
	_last_time = test_time;	// remember time
	current_phase = cruise;
	}

	// CRUISE PHASE
	// here have to keep in this phase @15A until we
	// get to 3.6V per cell
	if(current_phase == cruise) {
		if(battery.voltage_v / 12 < low_battery_v) {
			test_debug.data[phase_counter] = percent_land;
			_last_time = test_time;
			update_all_outputs(percent_land);
			test_debug.id = 4;
			current_phase = land;
			_last_time = test_time;
		}
	}

	if(test_time - _last_time > time_land && current_phase == land)
	{
		// put throttle back to 70A
		test_debug.data[phase_counter] = percent_done;
		test_debug.id = 5;
		update_all_outputs(-1);
		current_phase = finished;

	}




	// if new battery data
	if(_battery_sub.updated()) {


		if(_battery_sub.copy(&battery)){

			// test - copy battery voltage
			test_debug.data[voltage] = battery.voltage_v;
			// test - copy battery current
			test_debug.data[current] = battery.current_a;
			// test - adjust throt

			// Adjust throttle based on current


		}



	}

	if(test_time - _last_debug_time > 1e6) {

		_last_debug_time = test_time;
		counter++;


		test_debug.data[seconds_counter] = counter;


	}

	_debug_pub.publish(test_debug);



}




extern "C" __EXPORT int vesta_battery_test_main(int argc, char *argv[])
{
	return VestaBatteryTest::main(argc, argv);
}



static px4::atomic<bool> thread_should_exit {false};

static VestaBatteryTest *Vesta_Battery_Test = nullptr;
int main(int argc, char *argv[])
{

	if(!strcmp(argv[1], "start")) {
		if(Vesta_Battery_Test != nullptr && Vesta_Battery_Test->is_running()){
			PX4_WARN("already running!!!");
			return 0;
		}
		Vesta_Battery_Test = new VestaBatteryTest();
	}

	if(!strcmp(argv[1], "stop")) {


		if (Vesta_Battery_Test == nullptr || !Vesta_Battery_Test->is_running()) {
			PX4_WARN("not running");
			/* this is not an error */
			return 0;
		}
		Vesta_Battery_Test->update_all_outputs(-1);


	}
}
