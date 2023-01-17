
// Init/Start/Stop headers
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>


// Publish and subscribe headers
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/log_message.h>
#include <uORB/topics/ulog_stream.h>




class MavlinkTerrain : public ModuleBase<MavlinkTerrain>, public ModuleParams, public px4::ScheduledWorkItem
{

public:
	MavlinkTerrain();
	~MavlinkTerrain() override = default;

	static int task_spawn(int argc, char argv[]);
	static int custom_command(int argc, char argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	uORB::Subscription	_terrain_data{ORB_ID(mavlink_msg_terrain_data_decode)}
	uORB

	uORB::Publication<camera_capture_s>	_capture_pub{ORB_ID(camera_capture)}
	uORB::Publication<uot



}








