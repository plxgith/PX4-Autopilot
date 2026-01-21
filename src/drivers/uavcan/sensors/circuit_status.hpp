#pragma once

#include "sensor_bridge.hpp"
#include <uavcan/equipment/power/CircuitStatus.hpp>
/* This is supposed to be outgenerated, DONT KNOW IF I NEED THIS
	here is the battery reference
	#include <ardupilot/equipment/power/BatteryInfoAux.hpp>
*/
// #include <ardupilot/equipment/power/>


// Used to get access to class ModuleParams
#include <px4_platform_common/module_params.h>

class UavcanCircuitStatusBridge : public UavcanSensorBridgeBase, public ModuleParams
{
public:

	static const char *const NAME;

	UavcanCircuitStatusBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	void circuit_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus> &msg);

	typedef uavcan::MethodBinder<UavcanCircuitStatusBridge *,
	       void (UavcanCircuitStatusBridge::*)
	       (const uavcan::ReceivedDataStructure<uavcan::equipment::power::CircuitStatus> &) >
	       CircuitStatusInfoCbBinder;

	uavcan::Subscriber<uavcan::equipment::power::CircuitStatus, CircuitStatusInfoCbBinder> _sub_circuit_status;

};


