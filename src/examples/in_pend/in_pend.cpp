/* Inverse pendulum constructed like MulticopterRateController */
// Not going into the inner loop


#include "in_pend.hpp"

//using namespace matrix;
using namespace time_literals;

inPend::inPend():
		ModuleParams(nullptr),
		WorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	PX4_INFO("Constructor Done");

}



inPend::~inPend()
{


}


bool inPend::init()
{

	if(!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angualar_velocity callback registration failed");
		PX4_INFO("CallBack Error");

		return false;
	}

	//ScheduleOnInterval(1000000_us);
	//SCHED_PRIORITY_POSITION_CONTROL;
	return true;
}


void inPend::Run()
{

	PX4_INFO("In Run");
	if(should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;

	}

	// run controller on gyro changes
	vehicle_angular_velocity_s angular_velocity;

	PX4_INFO("In loop");

	// check if there's new gyro data, and if so, copy data to angular velocity
	if(_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		PX4_INFO("In updated vehicle angular velocity loop");

		//px4_delay(1000000);

	}

	while(!should_exit())
	{
		PX4_INFO("In should exit");
		//px4_sleep(1000);
	}
}

int inPend::task_spawn(int argc, char *argv[])
{
	/*
	px4_main_t entry_point = (px4_main_t)&run_trampoline();

	int task_id = px4_task_spawn_cmd("In Pend",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT,
					1500,
					entry_point,
					(char *const *)argv);

	*/

	PX4_INFO("Creating instance");
	inPend *instance = new inPend();

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

int inPend::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int inPend::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int in_pend_main(int argc, char *argv[])
{
	PX4_INFO("Called from NuttX");

	return inPend::main(argc, argv);
}
