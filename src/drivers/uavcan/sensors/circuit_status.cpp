#include "circuit_status.hpp"

// #include <lib/ecl/geo/geo.h>
#include <px4_defines.h>

const char *const UavcanCircuitStatusBridge::NAME = "circuit_status";

UavcanCircuitStatusBridge::UavcanCircuitStatusBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_circuit_status", ORB_ID(circuit_status)),
	ModuleParams(nullptr),
	_sub_circuit_status(node)
{

}

int UavcanCircuitStatusBridge::init()
{
	int res = _sub_circuit_status.start(CircuitStatusInfoCbBinder(this, &UavcanCircuitStatusBridge::circuit_status_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void circuit_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus> &msg)
{
	// TODO[TIN]
}
