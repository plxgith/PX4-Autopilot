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

void UavcanCircuitStatusBridge::circuit_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus> &msg)
{
	circuit_status_s report{};
	report.timestamp = hrt_absolute_time();
	report.voltage = msg.voltage;
	report.current = msg.current;
	// report.power_w   = msg.voltage * msg.current;
	report.circuit_id = msg.circuit_id;
	// report.status_flags = msg.status;

	// _sensor_pub.publish(report);
	publish(msg.getSrcNodeID().get(), &report);
}
