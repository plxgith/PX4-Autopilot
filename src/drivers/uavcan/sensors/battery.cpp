/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "battery.hpp"

#include <lib/geo/geo.h>
#include <px4_defines.h>

// for fetching parameters
#include <lib/parameters/param.h>


const char *const UavcanBatteryBridge::NAME = "battery";

UavcanBatteryBridge::UavcanBatteryBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_battery", ORB_ID(battery_status)),
	ModuleParams(nullptr),
	_sub_battery(node),
	_sub_battery_aux(node),
	_warning(battery_status_s::BATTERY_WARNING_NONE),
	_last_timestamp(0)
{
	_throttle_filter.setParameters(1.f, 1.f);	// try one second
	_current_filter_a.setParameters(1.f, 1.f);
	// parameters init
	//param_get(_param_bat1_full_voltage, &_bat1_full_voltage);
	//param_get(_param_bat1_full_wh, &bat1_full_wh);
	// param_get(_param_bat2_full_voltage, &bat2_full_voltage);
	// param_get(_param_bat2_full_wh, &bat2_full_wh);
	_params_handles._bat1_volt_drop = param_find("UAVCAN_BAT1_V_D");
	param_get(_params_handles._bat1_volt_drop, &_test);

	// _test = param_find("UAVCAN_TEST");
	// _bat1_capacity = param_find("BAT1_CAPACITY");
	// _bat1_num_of_cells = param_find("BAT1_N_CELLS");
	// _bat1_v_drop = param_find("BAT1_V_LOAD_DROP");
	// _bat1_v_empty = param_find("BAT1_V_EMPTY");
	// _bat1_v_charged = param_find("BAT1_V_CHARGED");

	// param_get("BAT_LOW_THR")
}

int UavcanBatteryBridge::init()
{
	int res = _sub_battery.start(BatteryInfoCbBinder(this, &UavcanBatteryBridge::battery_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	res = _sub_battery_aux.start(BatteryInfoAuxCbBinder(this, &UavcanBatteryBridge::battery_aux_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanBatteryBridge::battery_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg)
{
	uint8_t instance = 0;

	float full_voltage = 50.4;
	float full_wh  = 28.0;
	float full_mAh = 28000;
	float time_remaining = 0;



	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (battery_status[instance].id == msg.getSrcNodeID().get() || battery_status[instance].id == 0) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES) {
		return;
	}

	// get data from sensor
	battery_status[instance].timestamp = hrt_absolute_time();
	battery_status[instance].voltage_v = msg.voltage;
	battery_status[instance].voltage_filtered_v = msg.voltage;
	battery_status[instance].current_a = msg.current;
	battery_status[instance].current_filtered_a = msg.current;
	// battery_status[instance].current_average_a = msg.;

	if (battery_aux_support[instance] == false) {
		sumDischarged(battery_status[instance].timestamp, battery_status[instance].current_a);
		battery_status[instance].discharged_mah = _discharged_mah;
	}

	// calculate remaiming battery

	//battery_status[instance].remaining = calculate_remaining(msg.voltage, full_voltage) / 100.0; // between 0 and 1
	// battery_status[instance].remaining = estimate_state_of_charge_voltage_based(msg.voltage, msg.current);
	battery_status[instance].remaining = estimate_state_of_charge(msg.voltage, msg.current);
	// battery_status[instance].scale = msg.; // Power scaling factor, >= 1, or -1 if unknown
	//battery_status[instance].temperature = msg.temperature + CONSTANTS_ABSOLUTE_NULL_CELSIUS; // Kelvin to Celcius
	// battery_status[instance].cell_count = msg.;
	battery_status[instance].connected = true;
	battery_status[instance].source = msg.status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
	// battery_status[instance].priority = msg.;
	// battery_status[instance].capacity = msg.;
	battery_status[instance].full_charge_capacity_wh = msg.full_charge_capacity_wh;
	battery_status[instance].remaining_capacity_wh = 28000.0f;
	// battery_status[instance].cycle_count = msg.;
	//time_remaining  = calculate_time_remaining(msg.current);
	battery_status[instance].time_remaining_s = calculate_time_remaining(msg.current);
	// battery_status[instance].average_time_to_empty = msg.;
	battery_status[instance].serial_number = msg.model_instance_id;
	battery_status[instance].id = msg.getSrcNodeID().get();

	if (battery_aux_support[instance] == false) {
		// Mavlink 2 needs individual cell voltages or cell[0] if cell voltages are not available.
		battery_status[instance].voltage_cell_v[0] = msg.voltage;

		// Set cell count to 1 so the the battery code in mavlink_messages.cpp copies the values correctly (hack?)
		battery_status[instance].cell_count = 1;
	}

	// battery_status[instance].max_cell_voltage_delta = msg.;

	// battery_status[instance].is_powering_off = msg.;

	determineWarning(battery_status[instance].remaining);
	battery_status[instance].warning = _warning;

	if (battery_aux_support[instance] == false) {
		publish(msg.getSrcNodeID().get(), &battery_status[instance]);
	}
}

float
UavcanBatteryBridge::calculate_remaining(float voltage, float full_voltage)
{
	//_count ++;
	//PX4_INFO("In calculate remaining\n");
	return voltage / full_voltage * 100.0f;
}

float
UavcanBatteryBridge::calculate_hours_remaining(float remaining_capacity_wh, float voltage, float current)
{

	float time_remaining = NAN;
	if(current < 0) {
		return NAN;
	}

	// calculate remaining capacity in Ah
	float remaining_ah = remaining_capacity_wh / voltage;

	// calculate time remaining in hours
	float remaining_hours = remaining_ah / current;

	return remaining_hours;
}

float UavcanBatteryBridge::estimate_state_of_charge_voltage_based(const float voltage_v, const float current_a)
{
	float temp{0.0f};

	float cell_voltage = voltage_v / _bat1_num_of_cells;
	_temp_array.data[0] = _params_handles._bat1_volt_drop;
	// correct voltage for load drop
	actuator_controls_s actuator_controls{};
	_actuator_controls_0_sub.copy(&actuator_controls);
	const float throttle = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
	_temp_array.data[1] = throttle;

	_throttle_filter.update(throttle);
	temp = _throttle_filter.getState();
	_temp_array.data[2] = temp;

	// assume linear relation between throtle and voltage drop
	cell_voltage += throttle * _bat1_v_drop;
	_temp_array.data[3] = cell_voltage;


	//voltage based state of charge
	_state_of_charge_volt_based =  math::gradual(cell_voltage, _bat1_v_empty, _bat1_v_charged, 0.f, 1.f );
	_temp_array.data[4] = _state_of_charge_volt_based;

	// _temp_array.data[2] = _bat1_v_charged;

	// _temp_array.data[3] = _bat1_v_empty;
	// _temp_array.data[4] = _bat1_capacity;
	// _temp_array.data[5] = _bat1_num_of_cells;

	_debug_array_pub.publish(_temp_array);
	return _state_of_charge_volt_based;


}

float UavcanBatteryBridge::estimate_state_of_charge(const float voltage_v, const float current_a)
{
	estimate_state_of_charge_voltage_based(voltage_v, current_a);
	// choose which quantity is used for final report
	if (_bat1_capacity > 0.f) {
		// if capacity is known, fuse voltage measurements with used capacity
		const float weight_v = 3e-4f * (1 - _state_of_charge_volt_based);
		_temp_array.data[5] = weight_v;
		_state_of_charge = (1 - weight_v) * _state_of_charge + weight_v * _state_of_charge_volt_based;
		_temp_array.data[6] = _state_of_charge;
		// apply current capacity slope calculated using current
		_state_of_charge -= _discharged_mah_loop / _bat1_capacity;
		_temp_array.data[7] = _state_of_charge;
		_state_of_charge = math::max(_state_of_charge, 0.f);
		_temp_array.data[8] = _state_of_charge;
		const float state_of_charge_current_based = math::max(1.f - _discharged_mah / _bat1_capacity, 0.f);
		_temp_array.data[9] = state_of_charge_current_based;
		_state_of_charge = math::min(state_of_charge_current_based, _state_of_charge);
		_temp_array.data[10] = _state_of_charge;
	}
	else {
		_state_of_charge = _state_of_charge_volt_based;
	}

	_debug_array_pub.publish(_temp_array);
	return _state_of_charge;
	//return 100.0;

}

float UavcanBatteryBridge::calculate_time_remaining(float current_a)
{
	float time_remaining_s = NAN;
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.copy(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}
	_temp_array.data[11] = _armed;


	if(!PX4_ISFINITE(_current_filter_a.getState()) || _current_filter_a.getState() < FLT_EPSILON) {
		_current_filter_a.reset(10.0f);
	}
	if(_armed && PX4_ISFINITE(current_a)) {
		_current_filter_a.update(fmaxf(current_a, 0.f));
	}
	_temp_array.data[12] = _current_filter_a.getState();


	// Remaining time estimation only possible with capacity
	if(_bat1_capacity > 0.f) {
		const float remaining_capacity_mah = _state_of_charge * _bat1_capacity;
		_temp_array.data[13] = remaining_capacity_mah;

		const float current_ma = fmaxf(_current_filter_a.getState() * 1e3f, FLT_EPSILON);
		time_remaining_s = remaining_capacity_mah / current_ma * 3600.f;
	}

	_temp_array.data[14] = time_remaining_s;
	return time_remaining_s;
}

void
UavcanBatteryBridge::battery_aux_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryInfoAux>
					&msg)
{
	uint8_t instance = 0;

	for (instance = 0; instance < battery_status_s::MAX_INSTANCES; instance++) {
		if (battery_status[instance].id == msg.getSrcNodeID().get()) {
			break;
		}
	}

	if (instance >= battery_status_s::MAX_INSTANCES) {
		return;
	}

	battery_aux_support[instance] = true;

	battery_status[instance].discharged_mah = (battery_status[instance].full_charge_capacity_wh -
			battery_status[instance].remaining_capacity_wh) / msg.nominal_voltage *
			1000;
	battery_status[instance].cell_count = math::min((uint8_t)msg.voltage_cell.size(), (uint8_t)14);
	battery_status[instance].cycle_count = msg.cycle_count;
	battery_status[instance].over_discharge_count = msg.over_discharge_count;
	battery_status[instance].nominal_voltage = msg.nominal_voltage;
	battery_status[instance].time_remaining_s = math::isZero(battery_status[instance].current_a) ? 0 :
			(battery_status[instance].remaining_capacity_wh /
			 battery_status[instance].nominal_voltage / battery_status[instance].current_a * 3600);
	battery_status[instance].is_powering_off = msg.is_powering_off;

	for (uint8_t i = 0; i < battery_status[instance].cell_count; i++) {
		battery_status[instance].voltage_cell_v[i] = msg.voltage_cell[i];
	}

	publish(msg.getSrcNodeID().get(), &battery_status[instance]);
}

void
UavcanBatteryBridge::sumDischarged(hrt_abstime timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dt.
	if (_last_timestamp != 0) {
		const float dt = (timestamp - _last_timestamp) / 1e6;
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	_last_timestamp = timestamp;
}

void
UavcanBatteryBridge::determineWarning(float remaining)
{
	// propagate warning state only if the state is higher, otherwise remain in current warning state
	if (remaining < _param_bat_emergen_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_EMERGENCY)) {
		_warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (remaining < _param_bat_crit_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
		_warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (remaining < _param_bat_low_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_LOW)) {
		_warning = battery_status_s::BATTERY_WARNING_LOW;
	}
}
