#include "output_tests.hpp"

OutputTest::OutputTest():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)

{

}

OutputTest::~OutputTest
{


}

int OutputTest::task_spawn(int argc, char *argv[])
{

	// Create instance of the class
	OutputTest *instance = new OutputTest();

	if(instance) {
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

int OutputTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");

}

int OutputTest::print_usage(const char *reason)
{
	if(reason){
		PX4_WARN("%s\n", reason);
	}
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
output_test is my custom class for testing various parts of the system.
)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_NAME("test_functions");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

bool OutputTest::init()
{


}
