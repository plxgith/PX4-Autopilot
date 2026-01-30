#ifndef CIRCUIT_STATUS_HPP
#define CIRCUIT_STATUS_HPP

#include <uORB/topics/circuit_status.h>

class MavlinkStreamCircuitStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCircuitStatus(mavlink); }

	static constexpr const char *get_name_static() { return "CIRCUIT_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CIRCUIT_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _circuit_status_subs.advertised() ? MAVLINK_MSG_ID_CIRCUIT_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCircuitStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _circuit_status_subs{ORB_ID::circuit_status};

	bool send() override
	{
		circuit_status_s circuit_status_data;

		if (_circuit_status_subs.update(&circuit_status_data)) {
			mavlink_circuit_status_t msg {};
			msg.timestamp = hrt_absolute_time();
			msg.id = circuit_status_data.circuit_id;
			msg.voltage = circuit_status_data.voltage;
			msg.current = circuit_status_data.current;
			// msg.flags = circuit_status_data.flags;

			mavlink_msg_circuit_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};



#endif // CIRCUIT_STATUS_HPP
