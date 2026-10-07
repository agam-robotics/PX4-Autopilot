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

/**
 * @file SerialSensorOut.hpp
 *
 * Output of a sensor node's optical flow, distance and IMU measurements to a
 * flight controller over a serial port. Sends MAVLink to PX4 and ArduPilot and
 * MSP to INAV and Betaflight, detecting which one is connected.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_optical_flow.h>

#include <common/mavlink.h>

using namespace time_literals;

class SerialSensorOut : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	SerialSensorOut(const char *port, uint32_t baudrate);
	~SerialSensorOut() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	enum class Peer : uint8_t {
		Unknown,	///< nothing heard yet, no output
		Mavlink,	///< MAVLink of unknown origin, or nothing heard within the timeout
		PX4,
		ArduPilot,
		INAV,
		Betaflight,
	};

	static constexpr hrt_abstime DETECT_TIMEOUT{10_s};	///< fall back to MAVLink after this long without a reply

	void Run() override;
	void parameters_update();

	bool sends_mavlink() const { return _peer == Peer::Mavlink || _peer == Peer::PX4 || _peer == Peer::ArduPilot; }

	bool sends_msp() const { return _peer == Peer::INAV || _peer == Peer::Betaflight; }

	void set_peer(Peer peer, bool detected);
	static const char *peer_name(Peer peer);

	// detection
	void poll_rx();
	void parse_mavlink(uint8_t c);
	void parse_msp(uint8_t c);
	void send_msp_fc_variant_request();

	// MAVLink
	void send_heartbeat();
	void send_mavlink_optical_flow(const vehicle_optical_flow_s &flow);
	void send_mavlink_distance_sensor(const distance_sensor_s &dist);
	void send_highres_imu(const vehicle_imu_s &imu);
	void send_mavlink_message();

	// MSP
	void send_msp_sensors(const vehicle_optical_flow_s &flow);
	void send_msp(uint16_t cmd, const void *payload, uint16_t size);

	void write(const uint8_t *buffer, uint16_t len);

	device::Serial _uart;

	uORB::SubscriptionCallbackWorkItem _vehicle_optical_flow_sub{this, ORB_ID(vehicle_optical_flow)};
	uORB::SubscriptionCallbackWorkItem _distance_sensor_sub{this, ORB_ID(distance_sensor)};
	uORB::SubscriptionInterval _vehicle_imu_sub{ORB_ID(vehicle_imu)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	Peer _peer{Peer::Unknown};
	bool _detected{false};	///< peer identified, stop listening

	hrt_abstime _start_time{0};
	hrt_abstime _last_probe{0};
	hrt_abstime _last_heartbeat{0};

	distance_sensor_s _distance{};

	// MSP flow counts are integers, carry the rounding remainder so no motion is lost
	float _flow_residual[2] {};

	// MAVLink
	mavlink_message_t _msg{};
	uint8_t _buf[MAVLINK_MAX_PACKET_LEN] {};
	mavlink_message_t _rx_buf{};
	mavlink_message_t _rx_msg{};
	mavlink_status_t _rx_status{};
	mavlink_status_t _rx_msg_status{};

	// MSP v1 reply parser
	enum class MspState : uint8_t { Idle, M, Direction, Size, Command, Payload, Checksum };
	MspState _msp_state{MspState::Idle};
	uint8_t _msp_size{0};
	uint8_t _msp_cmd{0};
	uint8_t _msp_len{0};
	uint8_t _msp_checksum{0};
	uint8_t _msp_payload[4] {};

	perf_counter_t _sent_perf{perf_alloc(PC_COUNT, MODULE_NAME": sent")};
	perf_counter_t _dropped_perf{perf_alloc(PC_COUNT, MODULE_NAME": dropped")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SSO_PROTO>) _param_sso_proto,
		(ParamInt<px4::params::SSO_IMU_RATE>) _param_sso_imu_rate,
		(ParamInt<px4::params::SSO_MAV_SYSID>) _param_sso_mav_sysid,
		(ParamInt<px4::params::SSO_MAV_COMPID>) _param_sso_mav_compid
	)
};
