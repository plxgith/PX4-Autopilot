
#include "mad_max.hpp"


MadMax::MadMax()
{
	// Do nothing in constructor

}


MadMax *MadMax::instantiate(int argc, char *argv[])	//returns pointer type madmax
{
	return new MadMax();	// constructor call
}

int MadMax::task_spawn(int argc, char *argv[])
{
	px4_main_t entry_point = (px4_main_t)&run_trampoline();	// Entry point for px4_task_spawn_cmd() if the module runs in its own thread.

	int _task_id = px4_task_spawn_cmd("Mad Max", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1500, entry_point, (char *const *)argv);


	if(_task_id < 0) {
		PX4_INFO("Mad MAx module instantiation failed");
		_task_id = -1;
		return -errno;
	}
	else {
		return PX4_OK;
	}
}

void MadMax::run()
{
	static vehicle_local_position_s v_position{};
	hrt_abstime now = hrt_absolute_time();

	// New topic data had been received
	if(_vehicle_local_position_sub.update(&v_position)) {


		const Vector2f vehicle_xy_vel{v_position.xv, v_position_vy};
		const float vehicle_speed_m_s = vehicle_xy_vel.norm();



	}

	px4_sleep(1000_ms);


}


int MadMax::print_usage(const char *reason)
{
	return 0;
}

int MadMax::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command");
}


// Main entry point function for PX4 Nuttx System
int mad_max_main(int argc, char *argv[])
{
	return MadMax::main(argc, argv);
}
