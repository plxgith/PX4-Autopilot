#include "circuit_status.hpp"
#include <px4_defines.h>

const char *const UavcanCircuitStatusBridge::NAME = "circuit_status";

// Constructor
// - "uavcan_circuit_status" is just a debug label shown in logs
// - ORB_ID(circuit_status) tells the base class which uORB topic to publish
// - _sub_circuit_status(node) initializes the UAVCAN subscriber with the shared node instance
UavcanCircuitStatusBridge::UavcanCircuitStatusBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_circuit_status", ORB_ID(circuit_status)),
	_sub_circuit_status(node)
{}

// init()
// Called when the UAVCAN driver starts up.
// Registers this bridge’s subscriber callback with libuavcan.
int UavcanCircuitStatusBridge::init()
{
	int res = _sub_circuit_status.start(CircuitStatusInfoCbBinder(this, &UavcanCircuitStatusBridge::circuit_status_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

// Callback: executed each time a UAVCAN CircuitStatus message arrives.
void UavcanCircuitStatusBridge::circuit_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus> &msg)
{
	// Create a uORB message instance
	circuit_status_s report{};

	// Fill in fields using data from the UAVCAN message
	report.timestamp  = hrt_absolute_time();
	report.circuit_id = msg.circuit_id;
	report.voltage    = msg.voltage;
	report.current    = msg.current;
	report.is_fan_working	= msg.is_fan_working;

	// Convert UAVCAN status bits into our local flags field
	report.flags = msg.error_flags;

	// Publish the filled structure to uORB so PX4 modules / QGC can read it
	publish(msg.getSrcNodeID().get(), &report);
}
