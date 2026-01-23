#ifndef CIRCUIT_STATUS_HPP
#define CIRCUIT_STATUS_HPP

#include <uORB/topics/circuit_status.h>

class MavlinkStreamCircuitStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCircuitStatus(mavlink); }

	static constexpr const char *get_name_static() { return "CIRCUIT_STATUS"; }
	static constexpr uint16_t get_id_static() { return 0;/*TODO[TIN]*/ }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		;
		//TODO[TIN]
	}

private:
	explicit MavlinkStreamCircuitStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<circuit_status_s> _circuit_status_subs{ORB_ID::circuit_status};

	bool send() override
	{
		bool update = false;

		//TODO[TIN]
	}
};



#endif // CIRCUIT_STATUS_HPP
