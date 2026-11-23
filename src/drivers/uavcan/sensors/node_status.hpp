#pragma once

#include "sensor_bridge.hpp"
#include <uavcan/protocol/NodeStatus.hpp>
#include <uORB/topics/node_status.h>

class UavcanNodeStatusBridge : public UavcanSensorBridgeBase, public ModuleParams
{
public:

	static const char *const NAME;

	UavcanNodeStatusBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	void node_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus> &msg);

	typedef uavcan::MethodBinder<UavcanNodeStatusBridge *,
	       void (UavcanNodeStatusBridge::*)
	       (const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus> &) >
	       NodeStatusInfoCbBinder;

	uavcan::Subscriber<uavcan::protocol::NodeStatus, NodeStatusInfoCbBinder> _sub_node_status;

};

