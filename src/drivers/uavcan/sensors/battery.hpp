/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alex@arkelectron.com>
 */

#pragma once

#include "sensor_bridge.hpp"
#include <uORB/topics/battery_status.h>
#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <ardupilot/equipment/power/BatteryInfoAux.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

// include filtering lib
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
// include actuator controls topic to compensate for full throttle
// voltage drop
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
// include uORB topic for debugging
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_array.h>
#include <uORB/Publication.hpp>
// include math lib for interpolations, ...
#include <lib/battery/battery.h>
// for printing
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
// for vehicle status
#include <uORB/topics/vehicle_status.h>
// for fetching parameters
#include <lib/parameters/param.h>

class UavcanBatteryBridge : public UavcanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	UavcanBatteryBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	void battery_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg);
	void battery_aux_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryInfoAux> &msg);
	void sumDischarged(hrt_abstime timestamp, float current_a);
	void determineWarning(float remaining);
	float calculate_remaining(float voltage, float full_voltage);
	float calculate_hours_remaining(float remaining_capacity_wh, float voltage, float current);
	float calculate_time_remaining(float current_a);
	float estimate_state_of_charge(const float voltage_v, const float curent_a);
	float estimate_state_of_charge_voltage_based(const float voltage_v, const float current_a);

	typedef uavcan::MethodBinder < UavcanBatteryBridge *,
		void (UavcanBatteryBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &) >
		BatteryInfoCbBinder;
	typedef uavcan::MethodBinder < UavcanBatteryBridge *,
		void (UavcanBatteryBridge::*)
		(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryInfoAux> &) >
		BatteryInfoAuxCbBinder;

	uavcan::Subscriber<uavcan::equipment::power::BatteryInfo, BatteryInfoCbBinder> _sub_battery;
	uavcan::Subscriber<ardupilot::equipment::power::BatteryInfoAux, BatteryInfoAuxCbBinder> _sub_battery_aux;

	// subscription for  multirotor actuator controls
	uORB::Subscription _actuator_controls_0_sub{ORB_ID(actuator_controls_0)};
	// subscription for vehicle status
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr,
		(ParamFloat<px4::params::BAT_CRIT_THR>) _param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>) _param_bat_emergen_thr//,
		// (ParamFloat<px4::params::BAT1_FULL_V>) _param_bat1_full_voltage
		//(ParamFloat<px4::params::BAT1_FULL_WH>) _param_bat1_full_wh
	)

	// debug publication
	uORB::Publication<debug_value_s> _debug_pub{ORB_ID(debug_value)};
	uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};

	// additional variables for calculations
	float _voltage{0.f};
	AlphaFilter<float> _current_filter_a;
	AlphaFilter<float> _throttle_filter;
	//AlphaFilter<float> _voltage_filter_v;

	bool _battery_initialised = false;

	float _discharged_mah{0.f};
	float _discharged_mah_loop{0.f};

	float _state_of_charge_volt_based{-1.f};
	float _state_of_charge{1.f};

	float _test{0.0f};
	float _bat1_v_drop{0.3f};
	float _bat1_capacity{1600.0f};	// Cmarko
	int32_t _bat1_num_of_cells{3};	// Cmarko
	float _bat1_v_empty{3.5f};
	float _bat1_v_charged{4.0f};

	int _count = 1;
	debug_array_s _temp_array{0};
	bool _armed{false};


	uint8_t _warning;
	hrt_abstime _last_timestamp;
	battery_status_s battery_status[battery_status_s::MAX_INSTANCES] {};
	bool battery_aux_support[battery_status_s::MAX_INSTANCES] {};

protected:
	struct {
		param_t _bat1_volt_drop;
		param_t _bat1_capacity;
		param_t _bat1_num_of_cells;
		param_t _bat1_v_empty;
		param_t _bat1_v_charged;
	}_params_handles{};


};
