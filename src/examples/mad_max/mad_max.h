/* Module that runs as a task, reads speed, and givesmessages*/


/* Headers for work item and Modules*/
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


/* Headers for time and perf counters*/
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>




extern "C" __EXPORT int mad_max_main(int argc, char *argv[]);

// Inheritance to start a module; px4:: - namespace px4(to prevent clashing with nuttX??)
class MadMax : public ModuleBase<MadMax>, public px4::ScheduledWorkItem
{
public:
	/* Constructor and Deconstructor */
	MadMax();
	~MadMax() override;
	/**
	 * @see ModuleBase
	 *
	 */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);


	bool init();

	int print_status() override;


private:
	void Run() override;

};

