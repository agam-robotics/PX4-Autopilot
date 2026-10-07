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
#include <string.h>

#include <lib/mathlib/mathlib.h>
#include <lib/rc/common_rc.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

namespace
{
// MSP v1 request answered by INAV ("INAV") and Betaflight ("BTFL")
constexpr uint8_t MSP_FC_VARIANT = 2;

// MSP v2 sensor messages, same ID and layout in INAV and Betaflight
constexpr uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;
constexpr uint16_t MSP2_SENSOR_OPTIC_FLOW = 0x1F02;

// Flow counts per radian each expects: INAV divides by its default opflow_scale (10.5) to get deg/s,
// Betaflight (MicoAir MTF-01 convention) by 200 to get rad/s
constexpr float INAV_FLOW_SCALE = 10.5f * 180.f / M_PI_F;
constexpr float BETAFLIGHT_FLOW_SCALE = 200.f;
}

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

	parameters_update();

	switch (_param_sso_proto.get()) {
	case 1: set_peer(Peer::Mavlink, true); break;

	case 2: set_peer(Peer::INAV, true); break;

	case 3: set_peer(Peer::Betaflight, true); break;

	default: break; // auto-detect
	}

	if (!_vehicle_optical_flow_sub.registerCallback() || !_distance_sensor_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_start_time = hrt_absolute_time();
	ScheduleNow();
	return true;
}

void SerialSensorOut::parameters_update()
{
	updateParams();

	if (_param_sso_imu_rate.get() > 0) {
		_vehicle_imu_sub.set_interval_us(1_s / _param_sso_imu_rate.get());
	}
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
		parameters_update();
	}

	const hrt_abstime now = hrt_absolute_time();

	if (!_detected) {
		poll_rx();

		if (!_detected && (now - _last_probe >= 1_s)) {
			send_msp_fc_variant_request();
			_last_probe = now;
		}

		if ((_peer == Peer::Unknown) && (now - _start_time >= DETECT_TIMEOUT)) {
			// transmit-only wiring or a silent flight controller, keep listening
			set_peer(Peer::Mavlink, false);
		}
	}

	if (_distance_sensor_sub.update(&_distance) && sends_mavlink()) {
		send_mavlink_distance_sensor(_distance);
	}

	vehicle_optical_flow_s flow;

	if (_vehicle_optical_flow_sub.update(&flow)) {
		if (sends_mavlink()) {
			send_mavlink_optical_flow(flow);

		} else if (sends_msp()) {
			send_msp_sensors(flow);
		}
	}

	if (sends_mavlink()) {
		vehicle_imu_s imu;

		if ((_param_sso_imu_rate.get() > 0) && _vehicle_imu_sub.update(&imu)) {
			send_highres_imu(imu);
		}

		if (now - _last_heartbeat >= 1_s) {
			send_heartbeat();
			_last_heartbeat = now;
		}
	}

	// keep the heartbeat and detection going while no sensor data arrives
	ScheduleDelayed(_detected ? 1_s : 20_ms);
}

void SerialSensorOut::set_peer(Peer peer, bool detected)
{
	if (peer != _peer) {
		PX4_INFO("%s %s", detected ? "detected" : "assuming", peer_name(peer));
		_flow_residual[0] = 0.f;
		_flow_residual[1] = 0.f;
	}

	_peer = peer;
	_detected = detected;
}

const char *SerialSensorOut::peer_name(Peer peer)
{
	switch (peer) {
	case Peer::Mavlink: return "MAVLink";

	case Peer::PX4: return "PX4";

	case Peer::ArduPilot: return "ArduPilot";

	case Peer::INAV: return "INAV";

	case Peer::Betaflight: return "Betaflight";

	default: return "unknown";
	}
}

void SerialSensorOut::poll_rx()
{
	uint8_t buffer[64];

	// bounded, anything left is read on the next cycle
	for (int i = 0; (i < 8) && !_detected; i++) {
		const ssize_t n = _uart.read(buffer, sizeof(buffer));

		if (n <= 0) {
			break;
		}

		for (ssize_t j = 0; (j < n) && !_detected; j++) {
			parse_mavlink(buffer[j]);
			parse_msp(buffer[j]);
		}
	}
}

void SerialSensorOut::parse_mavlink(uint8_t c)
{
	if (mavlink_frame_char_buffer(&_rx_buf, &_rx_status, c, &_rx_msg, &_rx_msg_status) != MAVLINK_FRAMING_OK) {
		return;
	}

	if (_rx_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
		switch (mavlink_msg_heartbeat_get_autopilot(&_rx_msg)) {
		case MAV_AUTOPILOT_PX4:
			set_peer(Peer::PX4, true);
			return;

		case MAV_AUTOPILOT_ARDUPILOTMEGA:
			set_peer(Peer::ArduPilot, true);
			return;

		default:
			// not an autopilot, e.g. a GCS heartbeat forwarded to this link
			break;
		}
	}

	if (_peer == Peer::Unknown) {
		// MAVLink, but whose is not known until an autopilot heartbeat arrives
		set_peer(Peer::Mavlink, false);
	}
}

void SerialSensorOut::parse_msp(uint8_t c)
{
	switch (_msp_state) {
	case MspState::Idle:
		_msp_state = (c == '$') ? MspState::M : MspState::Idle;
		break;

	case MspState::M:
		_msp_state = (c == 'M') ? MspState::Direction : MspState::Idle;
		break;

	case MspState::Direction:
		_msp_state = (c == '>') ? MspState::Size : MspState::Idle;
		break;

	case MspState::Size:
		_msp_size = c;
		_msp_checksum = c;
		_msp_state = MspState::Command;
		break;

	case MspState::Command:
		_msp_cmd = c;
		_msp_checksum ^= c;
		_msp_len = 0;
		_msp_state = (_msp_size > 0) ? MspState::Payload : MspState::Checksum;
		break;

	case MspState::Payload:
		if (_msp_len < sizeof(_msp_payload)) {
			_msp_payload[_msp_len] = c;
		}

		_msp_checksum ^= c;

		if (++_msp_len == _msp_size) {
			_msp_state = MspState::Checksum;
		}

		break;

	case MspState::Checksum:
		if ((c == _msp_checksum) && (_msp_cmd == MSP_FC_VARIANT) && (_msp_size >= sizeof(_msp_payload))) {
			if (memcmp(_msp_payload, "INAV", sizeof(_msp_payload)) == 0) {
				set_peer(Peer::INAV, true);

			} else if (memcmp(_msp_payload, "BTFL", sizeof(_msp_payload)) == 0) {
				set_peer(Peer::Betaflight, true);
			}
		}

		_msp_state = MspState::Idle;
		break;
	}
}

void SerialSensorOut::send_msp_fc_variant_request()
{
	// size, command, checksum (size ^ command)
	const uint8_t request[] {'$', 'M', '<', 0, MSP_FC_VARIANT, MSP_FC_VARIANT};
	write(request, sizeof(request));
}

void SerialSensorOut::send_heartbeat()
{
	mavlink_heartbeat_t msg{};
	msg.type = MAV_TYPE_GENERIC;
	msg.autopilot = MAV_AUTOPILOT_INVALID;
	msg.system_status = MAV_STATE_ACTIVE;

	mavlink_msg_heartbeat_encode_chan(_param_sso_mav_sysid.get(), _param_sso_mav_compid.get(), MAVLINK_COMM_0, &_msg, &msg);
	send_mavlink_message();
}

void SerialSensorOut::send_mavlink_optical_flow(const vehicle_optical_flow_s &flow)
{
	const uint8_t sensor_id = _vehicle_optical_flow_sub.get_instance();
	const float distance = PX4_ISFINITE(flow.distance_m) ? flow.distance_m : -1.f;

	// each autopilot reads only one of the two, send both until it is known which one is connected
	if (_peer != Peer::ArduPilot) {
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
		send_mavlink_message();
	}

	if (_peer != Peer::PX4) {
		// angular rates, raw pixel counts are not available here
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

		mavlink_msg_optical_flow_encode_chan(_param_sso_mav_sysid.get(), _param_sso_mav_compid.get(), MAVLINK_COMM_0, &_msg,
						     &rate);
		send_mavlink_message();
	}
}

void SerialSensorOut::send_mavlink_distance_sensor(const distance_sensor_s &dist)
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
	send_mavlink_message();
}

void SerialSensorOut::send_highres_imu(const vehicle_imu_s &imu)
{
	if ((imu.delta_velocity_dt == 0) || (imu.delta_angle_dt == 0)) {
		return;
	}

	const float accel_dt_inv = 1e6f / imu.delta_velocity_dt;
	const float gyro_dt_inv = 1e6f / imu.delta_angle_dt;

	mavlink_highres_imu_t msg{};
	msg.time_usec = imu.timestamp_sample;
	msg.xacc = imu.delta_velocity[0] * accel_dt_inv;
	msg.yacc = imu.delta_velocity[1] * accel_dt_inv;
	msg.zacc = imu.delta_velocity[2] * accel_dt_inv;
	msg.xgyro = imu.delta_angle[0] * gyro_dt_inv;
	msg.ygyro = imu.delta_angle[1] * gyro_dt_inv;
	msg.zgyro = imu.delta_angle[2] * gyro_dt_inv;
	msg.fields_updated = 0b111111; // accel and gyro

	mavlink_msg_highres_imu_encode_chan(_param_sso_mav_sysid.get(), _param_sso_mav_compid.get(), MAVLINK_COMM_0, &_msg,
					    &msg);
	send_mavlink_message();
}

void SerialSensorOut::send_mavlink_message()
{
	const uint16_t len = mavlink_msg_to_send_buffer(_buf, &_msg);
	write(_buf, len);
}

void SerialSensorOut::send_msp_sensors(const vehicle_optical_flow_s &flow)
{
	// Sent with every flow sample: Betaflight rejects flow when its last range is older than ~50 ms
	struct __attribute__((packed)) {
		uint8_t quality;	// 0-255
		int32_t distance_mm;	// negative when out of range
	} range{};

	const bool range_valid = (hrt_elapsed_time(&_distance.timestamp) < 500_ms)
				 && (_distance.signal_quality != 0)
				 && (_distance.current_distance >= _distance.min_distance)
				 && (_distance.current_distance <= _distance.max_distance);

	range.quality = (_distance.signal_quality < 0) ? UINT8_MAX : (_distance.signal_quality * UINT8_MAX / 100);
	range.distance_mm = range_valid ? (int32_t)roundf(_distance.current_distance * 1e3f) : -1;
	send_msp(MSP2_SENSOR_RANGEFINDER, &range, sizeof(range));

	struct __attribute__((packed)) {
		uint8_t quality;	// 0-255
		int32_t motion_x;
		int32_t motion_y;
	} motion{};

	const float scale = (_peer == Peer::INAV) ? INAV_FLOW_SCALE : BETAFLIGHT_FLOW_SCALE;
	int32_t counts[2];

	for (int i = 0; i < 2; i++) {
		const float value = flow.pixel_flow[i] * scale + _flow_residual[i];
		counts[i] = (int32_t)roundf(value);
		_flow_residual[i] = value - counts[i];
	}

	motion.quality = flow.quality;
	motion.motion_x = counts[0];
	motion.motion_y = counts[1];
	send_msp(MSP2_SENSOR_OPTIC_FLOW, &motion, sizeof(motion));
}

void SerialSensorOut::send_msp(uint16_t cmd, const void *payload, uint16_t size)
{
	uint8_t frame[24];

	if (size + 9u > sizeof(frame)) {
		return;
	}

	// MSP v2 command: '$' 'X' '<' flags cmd(LE16) size(LE16) payload crc8_dvb_s2(flags..payload)
	frame[0] = '$';
	frame[1] = 'X';
	frame[2] = '<';
	frame[3] = 0;
	frame[4] = cmd & 0xFF;
	frame[5] = cmd >> 8;
	frame[6] = size & 0xFF;
	frame[7] = size >> 8;
	memcpy(&frame[8], payload, size);
	frame[8 + size] = crc8_dvb_s2_buf(&frame[3], 5 + size);

	write(frame, 9 + size);
}

void SerialSensorOut::write(const uint8_t *buffer, uint16_t len)
{
	// never block the work queue, drop the message if the TX buffer is full
	if ((_uart.txSpaceAvailable() >= len) && (_uart.write(buffer, len) == len)) {
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
	PX4_INFO("%s @ %" PRIu32 " baud, %s (%s)", _uart.getPort(), _uart.getBaudrate(), peer_name(_peer),
		 _detected ? "detected" : "assumed");
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
Sends a sensor node's optical flow, distance and IMU data to a flight controller over a serial port.

With SSO_PROTO set to auto, the flight controller is detected from what it sends:
- PX4 or ArduPilot: MAVLink HEARTBEAT, OPTICAL_FLOW_RAD (PX4) or OPTICAL_FLOW (ArduPilot), DISTANCE_SENSOR and HIGHRES_IMU.
- INAV or Betaflight: MSP rangefinder and optical flow messages, after the flight controller answers an MSP_FC_VARIANT request.

With nothing heard for 10 seconds, for example when only TX is wired, it falls back to MAVLink and keeps listening.

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
