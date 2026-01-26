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
		return _circuit_status_subs.advertised_count() * (MAVLINK_MSG_ID_CIRCUIT_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamCircuitStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<circuit_status_s> _circuit_status_subs{ORB_ID::circuit_status};

	bool send() override
	{
		bool update = false;

		for (int i = 0; i < _circuit_status_subs.size(); i++) {
			circuit_status_s circuit_status;

			if (_circuit_status_subs[i].update(&circuit_status)) {
				mavlink_circuit_status_t msg {};
				msg.id = i;
				msg.voltage = circuit_status.voltage;
				msg.current = circuit_status.current;

				mavlink_msg_circuit_status_send_struct(_mavlink->get_channel(), &msg);

				update = true;
			}
		}
	}
};



#endif // CIRCUIT_STATUS_HPP
