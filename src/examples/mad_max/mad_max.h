#pragma once

>
#include <px4_platform_common/module.h>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;
using namespace time_literals;


// Module start-stop handling function that gets called by Nuttx
extern "C" __EXPORT int mad_max_main(int argc, char *argv[]);


class MadMax : public ModuleBase<MadMax>
{
public:
	// Default constructor
	MadMax();
	~MadMax() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	static MadMax *instantiate(int argc, char *argv[]);



private:
	void run() override;

	// Subscription to the vehicle_local_position uORB topic
	uORB::Subscription		__vehicle_local_position{ORB_ID(vehicle_local_position)};



};
