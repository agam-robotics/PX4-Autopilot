/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "SerialSensorOut.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

ModuleBase::Descriptor SerialSensorOut::desc{task_spawn, custom_command, print_usage};

SerialSensorOut::SerialSensorOut(const char *port, uint32_t baudrate) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_uart(port, baudrate)
{
}

SerialSensorOut::~SerialSensorOut()
{
	perf_free(_sent_perf);
	perf_free(_dropped_perf);
}

bool SerialSensorOut::init()
{
	if (!_uart.open()) {
		PX4_ERR("failed to open %s", _uart.getPort());
		return false;
	}

	if (!_vehicle_optical_flow_sub.registerCallback() || !_distance_sensor_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	ScheduleNow();
	return true;
}

void SerialSensorOut::Run()
{
	if (should_exit()) {
		_vehicle_optical_flow_sub.unregisterCallback();
		_distance_sensor_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	vehicle_optical_flow_s flow;

	if (_vehicle_optical_flow_sub.update(&flow)) {
		send_optical_flow(flow);
	}

	distance_sensor_s dist;

	if (_distance_sensor_sub.update(&dist)) {
		send_distance_sensor(dist);
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_heartbeat >= 1_s) {
		send_heartbeat();
		_last_heartbeat = now;
	}

	// keep the heartbeat going while no sensor data arrives
	ScheduleDelayed(1_s);
}

void SerialSensorOut::send_heartbeat()
{
	mavlink_heartbeat_t msg{};
	msg.type = MAV_TYPE_GENERIC;
	msg.autopilot = MAV_AUTOPILOT_INVALID;
	msg.system_status = MAV_STATE_ACTIVE;

	mavlink_msg_heartbeat_encode_chan(_param_sso_mav_sysid.get(), _param_sso_mav_compid.get(), MAVLINK_COMM_0, &_msg, &msg);
	send_message();
}

void SerialSensorOut::send_optical_flow(const vehicle_optical_flow_s &flow)
{
	const uint8_t sensor_id = _vehicle_optical_flow_sub.get_instance();
	const float distance = PX4_ISFINITE(flow.distance_m) ? flow.distance_m : -1.f;

	// OPTICAL_FLOW_RAD (PX4)
	mavlink_optical_flow_rad_t rad{};
	rad.time_usec = flow.timestamp_sample;
	rad.sensor_id = sensor_id;
	rad.integration_time_us = flow.integration_timespan_us;
	rad.integrated_x = flow.pixel_flow[0];
	rad.integrated_y = flow.pixel_flow[1];
	rad.integrated_xgyro = flow.delta_angle[0];
	rad.integrated_ygyro = flow.delta_angle[1];
	rad.integrated_zgyro = flow.delta_angle[2];
	rad.quality = flow.quality;
	rad.distance = distance;

	mavlink_msg_optical_flow_rad_encode_chan(_param_sso_mav_sysid.get(), _param_sso_mav_compid.get(), MAVLINK_COMM_0, &_msg,
			&rad);
	send_message();

	// OPTICAL_FLOW (ArduPilot), as angular rates since raw pixel counts are not available here
	mavlink_optical_flow_t rate{};
	rate.time_usec = flow.timestamp_sample;
	rate.sensor_id = sensor_id;
	rate.quality = flow.quality;
	rate.ground_distance = distance;

	const float integration_time_s = flow.integration_timespan_us * 1e-6f;

	if (integration_time_s > FLT_EPSILON) {
		rate.flow_rate_x = flow.pixel_flow[0] / integration_time_s;
		rate.flow_rate_y = flow.pixel_flow[1] / integration_time_s;
	}

	if (distance > 0.f) {
		rate.flow_comp_m_x = rate.flow_rate_x * distance;
		rate.flow_comp_m_y = rate.flow_rate_y * distance;
	}

	mavlink_msg_optical_flow_encode_chan(_param_sso_mav_sysid.get(), _param_sso_mav_compid.get(), MAVLINK_COMM_0, &_msg, &rate);
	send_message();
}

void SerialSensorOut::send_distance_sensor(const distance_sensor_s &dist)
{
	mavlink_distance_sensor_t msg{};
	msg.time_boot_ms = dist.timestamp / 1000;

	switch (dist.type) {
	case MAV_DISTANCE_SENSOR_ULTRASOUND:
	case MAV_DISTANCE_SENSOR_INFRARED:
		msg.type = dist.type;
		break;

	default:
		msg.type = MAV_DISTANCE_SENSOR_LASER;
		break;
	}

	// clamp to the field ranges, converting an out-of-range float is undefined
	msg.current_distance = math::constrain(dist.current_distance * 1e2f, 0.f, (float)UINT16_MAX); // m to cm
	msg.min_distance = math::constrain(dist.min_distance * 1e2f, 0.f, (float)UINT16_MAX);
	msg.max_distance = math::constrain(dist.max_distance * 1e2f, 0.f, (float)UINT16_MAX);
	msg.covariance = math::constrain(dist.variance * 1e4f, 0.f, (float)UINT8_MAX); // m^2 to cm^2
	msg.orientation = dist.orientation;
	msg.horizontal_fov = dist.h_fov;
	msg.vertical_fov = dist.v_fov;

	for (int i = 0; i < 4; i++) {
		msg.quaternion[i] = dist.q[i];
	}

	// MAVLink reserves 0 for unknown and 1 for invalid
	if (dist.signal_quality < 0) {
		msg.signal_quality = 0;

	} else if (dist.signal_quality == 0) {
		msg.signal_quality = 1;

	} else {
		msg.signal_quality = dist.signal_quality;
	}

	mavlink_msg_distance_sensor_encode_chan(_param_sso_mav_sysid.get(), _param_sso_mav_compid.get(), MAVLINK_COMM_0, &_msg,
						&msg);
	send_message();
}

void SerialSensorOut::send_message()
{
	const uint16_t len = mavlink_msg_to_send_buffer(_buf, &_msg);

	// never block the work queue, drop the message if the TX buffer is full
	if (_uart.txSpaceAvailable() >= len && _uart.write(_buf, len) == len) {
		perf_count(_sent_perf);

	} else {
		perf_count(_dropped_perf);
	}
}

int SerialSensorOut::task_spawn(int argc, char *argv[])
{
	const char *port = nullptr;
	int baudrate = 115200;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			port = myoptarg;
			break;

		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				return PX4_ERROR;
			}

			break;

		default:
			print_usage("unrecognized option");
			return PX4_ERROR;
		}
	}

	if (port == nullptr) {
		print_usage("missing serial port (-d)");
		return PX4_ERROR;
	}

	SerialSensorOut *instance = new SerialSensorOut(port, baudrate);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int SerialSensorOut::print_status()
{
	PX4_INFO("%s @ %" PRIu32 " baud, sysid %" PRId32 " compid %" PRId32, _uart.getPort(), _uart.getBaudrate(),
		 _param_sso_mav_sysid.get(), _param_sso_mav_compid.get());
	perf_print_counter(_sent_perf);
	perf_print_counter(_dropped_perf);
	return 0;
}

int SerialSensorOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SerialSensorOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Transmit-only MAVLink output for sensor nodes connected to a flight controller over a serial port.

Sends HEARTBEAT, OPTICAL_FLOW_RAD, OPTICAL_FLOW and DISTANCE_SENSOR as each measurement is published.
It does not receive, so the node cannot be configured over this link.

### Examples
Normally started from the port configured with SSO_CONFIG. Manual start:
$ serial_sensor_out start -d /dev/ttyS0 -b 115200
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("serial_sensor_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 9600, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int serial_sensor_out_main(int argc, char *argv[])
{
	return ModuleBase::main(SerialSensorOut::desc, argc, argv);
}
