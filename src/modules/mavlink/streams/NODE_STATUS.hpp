#ifndef NODE_STATUS_HPP
#define NODE_STATUS_HPP

#include <uORB/topics/node_status.h>

class MavlinkStreamNodeStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamNodeStatus(mavlink); }

	static constexpr const char *get_name_static() { return "NODE_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NODE_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _node_status_subs.advertised() ? MAVLINK_MSG_ID_NODE_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamNodeStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _node_status_subs{ORB_ID::node_status};

	bool send() override
	{
		node_status_s node_status_data;

		if (_node_status_subs.update(&node_status_data)) {
			mavlink_node_status_t msg {};
			msg.timestamp = hrt_absolute_time();
			msg.uptime_sec = 0; //[TIN] Need to find or create a function for this
			msg.health = node_status_data.health;
			msg.mode = node_status_data.mode;
			msg.vendor_specific_status_code = node_status_data.vendor_specific_status_code;

			mavlink_msg_node_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};



#endif // NODE_STATUS_HPP
