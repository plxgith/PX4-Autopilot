#include "node_status.hpp"
#include <px4_defines.h>

const char *const UavcanNodeStatusBridge::NAME = "node_status";

UavcanNodeStatusBridge::UavcanNodeStatusBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_node_status", ORB_ID(node_status)),
	_sub_node_status(node)
{}

int UavcanNodeStatusBridge::init()
{
	int res = _sub_node_status.start(NodeStatusInfoCbBinder(this, &UavcanNodeStatusBridge::node_status_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanNodeStatusBridge::node_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::CircuitStatus> &msg)
{

}
