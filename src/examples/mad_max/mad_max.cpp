#include "mad_max"

MadMax::MadMax() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{

}

MadMax()::~MadMax()
{


}

MadMax::init()
{

	ScheduleOnInterval(2000_us);
	return true;
}

MadMax::print_status()
{
	return 0;
}




MadMax::task_spawn(intargc, char *argv[])
{
	MadMax *instance = new MadMax();

	if(instance) {	// if instance was succesfully created
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if(instance->init()) {
			return PX4_OK;
		}
	}
	else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}



MadMax::custom_command()
{
	return print_usage("unknown command");
}

MadMax::print_usage()
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



void MadMax::Run()
{
	if(should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}



	PX4_INFO("In loop");
}



// defined so that it can be called from the operating system
extern "C" __EXPORT int mad_max_main(int argc, char *argv[])
{
	// main funtion od module, defined in module.h, inherited
	// from ModuleBase
	return MadMax::main(int argc, argv);
}
