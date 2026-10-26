/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "GsLinkProtocol.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/gs_link_control.h>
#include <uORB/topics/gs_link_status.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>

#include <stddef.h>

using namespace time_literals;

class GsLink : public ModuleBase<GsLink>, public px4::ScheduledWorkItem
{
public:
	GsLink(const char *device, uint32_t baudrate);
	~GsLink() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void loadSubscriptions();
	bool openSerial();
	void readSerial();
	void parseRxBuffer();
	void handlePacket(const gs_link::Header &header, const uint8_t *payload);
	void handleRc(const gs_link::Header &header, const uint8_t *payload);
	void handleCommand(const gs_link::Header &header, const uint8_t *payload);
	void handleHeartbeat(const gs_link::Header &header, const uint8_t *payload);
	void sendTelemetry(hrt_abstime now);
	void sendHeartbeat(hrt_abstime now);
	void sendAck(uint8_t acked_class, uint8_t acked_id, uint16_t acked_seq,
		     gs_link::AckResult result, uint32_t echo_token);
	bool sendPacket(gs_link::PacketClass packet_class, uint8_t message_id, uint8_t flags,
			const void *payload, uint16_t payload_len);
	void publishControl(bool valid, hrt_abstime now);
	uint16_t buildStatusFlags(hrt_abstime now, bool attitude_valid, bool motor_valid, bool power_valid);
	bool heartbeatFresh(hrt_abstime now) const;
	bool rcDataFresh(hrt_abstime now) const;
	void resyncRxBuffer();

	static constexpr hrt_abstime kTelemetryInterval{20_ms};
	static constexpr hrt_abstime kHeartbeatInterval{50_ms};
	static constexpr hrt_abstime kHeartbeatTimeout{500_ms};
	static constexpr hrt_abstime kRcChannelsTimeout{300_ms};
	static constexpr hrt_abstime kRcTimeout{250_ms};
	static constexpr hrt_abstime kEstimatorTimeout{100_ms};

	device::Serial _serial;
	uint8_t _rx_buffer[gs_link::kMaxPacket] {};
	size_t _rx_len{0};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _gs_link_status_sub{ORB_ID(gs_link_status)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<gs_link_control_s> _control_pub{ORB_ID(gs_link_control)};

	actuator_armed_s _actuator_armed{};
	battery_status_s _battery_status{};
	esc_status_s _esc_status{};
	failsafe_flags_s _failsafe_flags{};
	gs_link_status_s _gate_status{};
	rc_channels_s _rc_channels{};
	vehicle_attitude_s _vehicle_attitude{};

	uint32_t _session_id{0};
	uint32_t _gs_session_id{0};
	uint16_t _tx_sequence{0};
	uint32_t _rx_packets{0};
	uint32_t _tx_packets{0};
	uint32_t _crc_errors{0};
	uint32_t _length_errors{0};
	uint32_t _version_errors{0};
	uint32_t _unknown_commands{0};
	uint32_t _open_failures{0};
	uint32_t _write_failures{0};
	int _last_write_errno{0};
	uint8_t _last_error{0};

	hrt_abstime _last_telemetry{0};
	hrt_abstime _last_heartbeat{0};
	hrt_abstime _last_gs_heartbeat{0};
	hrt_abstime _last_rc_arrival{0};

	bool _rc_data_valid{false};
	float _rc_roll{0.f};
	float _rc_pitch{0.f};
	float _rc_yaw{0.f};
	float _rc_throttle{0.f};
	uint8_t _rc_link_quality{255};
	uint16_t _rc_sequence{0};
};

extern "C" __EXPORT int gs_link_main(int argc, char *argv[]);
