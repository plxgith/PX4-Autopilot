

// Trying to get CAN battery to work
#pragma once

#include <battery/battery.h>
#include <parameters/param.h>

class CANBattery : public Battery
{
	public:
		CanBattery(int node_id, ModuleParams *parent, const int sample_interval_us,
		const uint8_t source, const uint8_t priority);



}
