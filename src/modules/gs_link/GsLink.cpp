/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "GsLink.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <errno.h>

#include <mathlib/math/Functions.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

namespace
{

constexpr float kRadToCentiDegrees{18000.f / 3.14159265358979323846f};

int16_t clampCentiDegrees(float value)
{
	if (!PX4_ISFINITE(value)) {
		return 0;
	}

	return static_cast<int16_t>(math::constrain(lroundf(value), -18000L, 18000L));
}

} // namespace

GsLink::GsLink(const char *device, uint32_t baudrate) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_serial(device, baudrate)
{
}

GsLink::~GsLink()
{
	_serial.close();
}

bool GsLink::init()
{
	_session_id = static_cast<uint32_t>(hrt_absolute_time()) ^ 0x9E3779B9u;

	// Open and close the serial device from the work queue thread. NuttX file
	// descriptors are task-group scoped, so an fd opened by the NSH task cannot
	// be used from the hp_default worker.
	ScheduleDelayed(1);
	return true;
}

bool GsLink::openSerial()
{
	if (_serial.isOpen()) {
		return true;
	}

	if (!_serial.open()) {
		++_open_failures;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::InternalError);
		PX4_ERR("failed to open %s at %lu baud", _serial.getPort(),
			(unsigned long)_serial.getBaudrate());
		return false;
	}

	// A stop/start cycle can inherit a stale TX buffer from the previous
	// owner. Start every GS-Link session with clean UART buffers.
	_serial.flush();

	_rx_len = 0;
	_last_telemetry = 0;
	_last_heartbeat = 0;
	_last_gs_heartbeat = 0;
	_last_rc_arrival = 0;
	_rc_data_valid = false;
	_gs_session_id = 0;
	_last_error = 0;
	_last_write_errno = 0;

	PX4_INFO("GS-Link started on %s at %lu baud, session %08lx",
		 _serial.getPort(), (unsigned long)_serial.getBaudrate(),
		 (unsigned long)_session_id);
	return true;
}

void GsLink::Run()
{
	if (should_exit()) {
		ScheduleClear();
		publishControl(false, hrt_absolute_time());
		_serial.close();
		exit_and_cleanup();
		return;
	}

	if (!openSerial()) {
		ScheduleDelayed(1_s);
		return;
	}

	loadSubscriptions();
	readSerial();
	parseRxBuffer();

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_telemetry >= kTelemetryInterval) {
		sendTelemetry(now);
		_last_telemetry = now;
		publishControl(rcDataFresh(now) && heartbeatFresh(now), now);
	}

	if (now - _last_heartbeat >= kHeartbeatInterval) {
		sendHeartbeat(now);
		_last_heartbeat = now;
	}

	ScheduleDelayed(1_ms);
}

void GsLink::loadSubscriptions()
{
	_actuator_armed_sub.update(&_actuator_armed);
	_battery_status_sub.update(&_battery_status);
	_esc_status_sub.update(&_esc_status);
	_failsafe_flags_sub.update(&_failsafe_flags);
	_gs_link_status_sub.update(&_gate_status);
	_rc_channels_sub.update(&_rc_channels);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
}

void GsLink::readSerial()
{
	uint8_t buffer[256];

	while (true) {
		const ssize_t received = _serial.read(buffer, sizeof(buffer));

		if (received <= 0) {
			break;
		}

		const size_t available = sizeof(_rx_buffer) - _rx_len;
		const size_t copy = math::min(static_cast<size_t>(received), available);
		memcpy(_rx_buffer + _rx_len, buffer, copy);
		_rx_len += copy;

		if (copy < static_cast<size_t>(received)) {
			// The parser will consume the valid prefix; drop one byte to make
			// room and resynchronize instead of permanently filling the buffer.
			resyncRxBuffer();
			break;
		}
	}
}

void GsLink::resyncRxBuffer()
{
	size_t start = 0;

	while (start + 1 < _rx_len
	       && !(_rx_buffer[start] == gs_link::kMagic0 && _rx_buffer[start + 1] == gs_link::kMagic1)) {
		++start;
	}

	if (start >= _rx_len) {
		_rx_len = 0;

	} else if (start > 0) {
		memmove(_rx_buffer, _rx_buffer + start, _rx_len - start);
		_rx_len -= start;
	}
}

void GsLink::parseRxBuffer()
{
	while (_rx_len >= gs_link::kHeaderSize + gs_link::kCrcSize) {
		if (_rx_buffer[0] != gs_link::kMagic0 || _rx_buffer[1] != gs_link::kMagic1) {
			resyncRxBuffer();
			continue;
		}

		gs_link::Header header{};
		memcpy(&header, _rx_buffer, sizeof(header));

		if (header.version != gs_link::kVersion) {
			++_version_errors;
			_last_error = static_cast<uint8_t>(gs_link::AckResult::BadVersion);
			memmove(_rx_buffer, _rx_buffer + 1, _rx_len - 1);
			--_rx_len;
			continue;
		}

		if (header.payload_len > gs_link::kMaxPayload) {
			++_length_errors;
			_last_error = static_cast<uint8_t>(gs_link::AckResult::BadLength);
			memmove(_rx_buffer, _rx_buffer + 1, _rx_len - 1);
			--_rx_len;
			continue;
		}

		const size_t total = gs_link::kHeaderSize + header.payload_len + gs_link::kCrcSize;

		if (_rx_len < total) {
			break;
		}

		uint16_t expected_crc{0};
		memcpy(&expected_crc, _rx_buffer + gs_link::kHeaderSize + header.payload_len,
		       sizeof(expected_crc));
		const uint16_t actual_crc = gs_link::crc16(_rx_buffer + 2,
					  gs_link::kHeaderSize - 2 + header.payload_len);

		if (actual_crc != expected_crc) {
			++_crc_errors;
			_last_error = 1;
			memmove(_rx_buffer, _rx_buffer + 1, _rx_len - 1);
			--_rx_len;
			continue;
		}

		handlePacket(header, _rx_buffer + gs_link::kHeaderSize);
		++_rx_packets;

		memmove(_rx_buffer, _rx_buffer + total, _rx_len - total);
		_rx_len -= total;
	}
}

void GsLink::handlePacket(const gs_link::Header &header, const uint8_t *payload)
{
	if (header.packet_class == static_cast<uint8_t>(gs_link::PacketClass::Data)) {
		if (header.message_id == static_cast<uint8_t>(gs_link::DataId::Rc)) {
			handleRc(header, payload);
		}

		return;
	}

	if (header.packet_class == static_cast<uint8_t>(gs_link::PacketClass::Command)) {
		handleCommand(header, payload);
	}
}

void GsLink::handleRc(const gs_link::Header &header, const uint8_t *payload)
{
	if (header.payload_len != sizeof(gs_link::RcPayload)) {
		++_length_errors;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::BadLength);
		return;
	}

	gs_link::RcPayload rc{};
	memcpy(&rc, payload, sizeof(rc));

	const bool valid = (rc.flags & gs_link::RcFlags::RcValid)
			   && rc.roll >= -10000 && rc.roll <= 10000
			   && rc.pitch >= -10000 && rc.pitch <= 10000
			   && rc.yaw >= -10000 && rc.yaw <= 10000
			   && rc.throttle >= -10000 && rc.throttle <= 10000;

	if (!valid) {
		_rc_data_valid = false;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::BadArgument);
		publishControl(false, hrt_absolute_time());
		return;
	}

	_rc_roll = static_cast<float>(rc.roll) / 10000.f;
	_rc_pitch = static_cast<float>(rc.pitch) / 10000.f;
	_rc_yaw = static_cast<float>(rc.yaw) / 10000.f;
	_rc_throttle = static_cast<float>(rc.throttle) / 10000.f;
	_rc_link_quality = rc.link_quality;
	_rc_sequence = header.sequence;
	_last_rc_arrival = hrt_absolute_time();
	_rc_data_valid = true;
	_last_error = 0;
	publishControl(heartbeatFresh(_last_rc_arrival), _last_rc_arrival);
}

void GsLink::handleHeartbeat(const gs_link::Header &header, const uint8_t *payload)
{
	if (header.payload_len != sizeof(gs_link::HeartbeatPayload)) {
		++_length_errors;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::BadLength);
		return;
	}

	gs_link::HeartbeatPayload heartbeat{};
	memcpy(&heartbeat, payload, sizeof(heartbeat));

	if (heartbeat.protocol_version != gs_link::kVersion) {
		++_version_errors;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::BadVersion);
		return;
	}

	_gs_session_id = heartbeat.session_id;
	_last_gs_heartbeat = hrt_absolute_time();
	_last_error = heartbeat.error_code;
}

void GsLink::handleCommand(const gs_link::Header &header, const uint8_t *payload)
{
	const auto command = static_cast<gs_link::CommandId>(header.message_id);

	switch (command) {
	case gs_link::CommandId::Ping: {
			if (header.payload_len != sizeof(gs_link::PingPayload)) {
				++_length_errors;
				sendAck(header.packet_class, header.message_id, header.sequence,
					gs_link::AckResult::BadLength, 0);
				return;
			}

			gs_link::PingPayload ping{};
			memcpy(&ping, payload, sizeof(ping));
			sendAck(header.packet_class, header.message_id, header.sequence,
				gs_link::AckResult::Ok, ping.token);
			return;
		}

	case gs_link::CommandId::Heartbeat:
		handleHeartbeat(header, payload);
		return;

	case gs_link::CommandId::Takeoff:
	case gs_link::CommandId::Land:
	case gs_link::CommandId::Abort: {
			// Communication-only firmware: validate the command envelope but
			// never enter a takeoff/landing state or touch the actuators.
			const size_t expected = command == gs_link::CommandId::Takeoff
						? sizeof(gs_link::TakeoffPayload)
						: command == gs_link::CommandId::Land
						? sizeof(gs_link::LandPayload)
						: sizeof(gs_link::AbortPayload);

			if (header.payload_len != expected) {
				++_length_errors;
				sendAck(header.packet_class, header.message_id, header.sequence,
					gs_link::AckResult::BadLength, 0);
				return;
			}

			uint32_t session_id{0};
			uint32_t token{0};

			if (command == gs_link::CommandId::Takeoff) {
				gs_link::TakeoffPayload takeoff{};
				memcpy(&takeoff, payload, sizeof(takeoff));
				session_id = takeoff.session_id;
				token = takeoff.command_token;

			} else if (command == gs_link::CommandId::Land) {
				gs_link::LandPayload land{};
				memcpy(&land, payload, sizeof(land));
				session_id = land.session_id;
				token = land.command_token;

			} else {
				gs_link::AbortPayload abort{};
				memcpy(&abort, payload, sizeof(abort));
				session_id = abort.session_id;
				token = abort.command_token;
			}

			sendAck(header.packet_class, header.message_id, header.sequence,
				session_id == _session_id ? gs_link::AckResult::Unsupported
				: gs_link::AckResult::StaleSession, token);
			return;
		}

	case gs_link::CommandId::Ack:
		return;

	default:
		++_unknown_commands;
		sendAck(header.packet_class, header.message_id, header.sequence,
			gs_link::AckResult::Unsupported, 0);
		return;
	}
}

void GsLink::sendAck(uint8_t acked_class, uint8_t acked_id, uint16_t acked_seq,
		     gs_link::AckResult result, uint32_t echo_token)
{
	gs_link::AckPayload ack{};
	ack.acked_class = acked_class;
	ack.acked_id = acked_id;
	ack.acked_seq = acked_seq;
	ack.result = static_cast<uint8_t>(result);
	ack.echo_token = echo_token;
	sendPacket(gs_link::PacketClass::Command, static_cast<uint8_t>(gs_link::CommandId::Ack),
		   gs_link::Flags::Response, &ack, sizeof(ack));
}

bool GsLink::sendPacket(gs_link::PacketClass packet_class, uint8_t message_id, uint8_t flags,
			const void *payload, uint16_t payload_len)
{
	if (payload_len > gs_link::kMaxPayload) {
		return false;
	}

	uint8_t packet[gs_link::kMaxPacket] {};
	gs_link::Header header{};
	header.version = gs_link::kVersion;
	header.packet_class = static_cast<uint8_t>(packet_class);
	header.message_id = message_id;
	header.flags = flags;
	header.sequence = _tx_sequence++;
	header.payload_len = payload_len;
	memcpy(packet, &header, sizeof(header));

	if (payload_len > 0) {
		memcpy(packet + gs_link::kHeaderSize, payload, payload_len);
	}

	const uint16_t crc = gs_link::crc16(packet + 2, gs_link::kHeaderSize - 2 + payload_len);
	memcpy(packet + gs_link::kHeaderSize + payload_len, &crc, sizeof(crc));

	const size_t total = gs_link::kHeaderSize + payload_len + gs_link::kCrcSize;
	size_t sent = 0;
	int retries = 0;
	int write_errno = 0;

	while (sent < total && retries < 4) {
		const ssize_t written = _serial.write(packet + sent, total - sent);

		if (written > 0) {
			sent += static_cast<size_t>(written);
			retries = 0;

		} else if (written < 0 && errno == EAGAIN) {
			write_errno = errno;
			++retries;
			px4_usleep(50);

		} else {
			write_errno = written < 0 ? errno : EIO;
			break;
		}
	}

	if (sent == total) {
		++_tx_packets;
		_last_error = 0;
		_last_write_errno = 0;
		return true;
	}

	++_write_failures;
	_last_write_errno = write_errno;
	_last_error = static_cast<uint8_t>(gs_link::AckResult::InternalError);
	return false;
}

uint16_t GsLink::buildStatusFlags(hrt_abstime now, bool attitude_valid, bool motor_valid,
				  bool power_valid)
{
	uint16_t flags{0};

	if (_actuator_armed.armed) {
		flags |= gs_link::StatusFlags::Armed;
	}

	if (_gate_status.gs_mode_active) {
		flags |= gs_link::StatusFlags::GsModeActive;
	}

	if (_gate_status.rc_valid) {
		flags |= gs_link::StatusFlags::RcValidStatus;
	}

	if (_gate_status.rc_switch_selected) {
		flags |= gs_link::StatusFlags::RcSwitchSelected;
	}

	if (_gate_status.rc_switch_valid) {
		flags |= gs_link::StatusFlags::RcSwitchValid;
	}

	if (_gate_status.failsafe || _failsafe_flags.manual_control_signal_lost
	    || _failsafe_flags.battery_unhealthy || _failsafe_flags.fd_critical_failure) {
		flags |= gs_link::StatusFlags::Failsafe;
	}

	if (attitude_valid) {
		flags |= gs_link::StatusFlags::AttitudeValid;
	}

	if (motor_valid) {
		flags |= gs_link::StatusFlags::MotorRpmValid;
	}

	if (power_valid) {
		flags |= gs_link::StatusFlags::PowerValid;
	}

	if (_battery_status.warning >= battery_status_s::WARNING_LOW) {
		flags |= gs_link::StatusFlags::BatteryWarning;
	}

	if (_gate_status.command_busy) {
		flags |= gs_link::StatusFlags::CommandBusy;
	}

	if (_gate_status.takeoff_active) {
		flags |= gs_link::StatusFlags::TakeoffActive;
	}

	if (_gate_status.landing_active) {
		flags |= gs_link::StatusFlags::LandingActive;
	}

	if (heartbeatFresh(now) && rcDataFresh(now)) {
		flags |= gs_link::StatusFlags::LinkOk;
	}

	if (_rc_channels.channel_count > 0 && !_rc_channels.signal_lost
	    && now >= _rc_channels.timestamp
	    && now - _rc_channels.timestamp <= kRcChannelsTimeout) {
		flags |= gs_link::StatusFlags::RcChannelsValid;
	}

	return flags;
}

void GsLink::sendTelemetry(hrt_abstime now)
{
	gs_link::TelemetryPayload telemetry{};
	telemetry.timestamp_ms = static_cast<uint32_t>(now / 1000);

	bool attitude_valid = false;

	if (_vehicle_attitude.timestamp_sample != 0
	    && now >= _vehicle_attitude.timestamp_sample
	    && now - _vehicle_attitude.timestamp_sample <= kEstimatorTimeout) {
		const matrix::Quatf q(_vehicle_attitude.q);

		if (PX4_ISFINITE(q(0)) && PX4_ISFINITE(q(1)) && PX4_ISFINITE(q(2)) && PX4_ISFINITE(q(3))
		    && q.norm_squared() > 0.25f) {
			const matrix::Eulerf euler(q);

			if (PX4_ISFINITE(euler.phi()) && PX4_ISFINITE(euler.theta())
			    && PX4_ISFINITE(euler.psi())) {
				telemetry.roll_cdeg = clampCentiDegrees(euler.phi() * kRadToCentiDegrees);
				// Protocol telemetry uses nose-down positive, matching the manual
				// pitch-stick sign; PX4 Euler pitch is nose-up positive.
				telemetry.pitch_cdeg = clampCentiDegrees(-euler.theta() * kRadToCentiDegrees);
				telemetry.yaw_cdeg = clampCentiDegrees(euler.psi() * kRadToCentiDegrees);
				attitude_valid = true;
			}
		}
	}

	bool motor_found[2] {false, false};

	if (_esc_status.timestamp != 0 && now >= _esc_status.timestamp
	    && now - _esc_status.timestamp <= kEstimatorTimeout) {
		for (int esc_index = 0; esc_index < _esc_status.esc_count
		     && esc_index < esc_status_s::CONNECTED_ESC_MAX; ++esc_index) {
			if ((_esc_status.esc_online_flags & (1u << esc_index)) == 0) {
				continue;
			}

			const int motor_index = static_cast<int>(_esc_status.esc[esc_index].actuator_function)
						- static_cast<int>(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1);

			if (motor_index >= 0 && motor_index < 2) {
				const float rpm = static_cast<float>(_esc_status.esc[esc_index].esc_rpm);

				if (PX4_ISFINITE(rpm) && rpm >= 0.f) {
					const uint16_t value = static_cast<uint16_t>(
								       math::constrain(lroundf(rpm), 0L, 65534L));
					telemetry.motor1_rpm = motor_index == 0 ? value : telemetry.motor1_rpm;
					telemetry.motor2_rpm = motor_index == 1 ? value : telemetry.motor2_rpm;
					motor_found[motor_index] = true;
				}
			}
		}
	}

	bool power_valid = false;

	if (_battery_status.timestamp != 0 && now >= _battery_status.timestamp
	    && now - _battery_status.timestamp <= 1_s) {
		if (PX4_ISFINITE(_battery_status.voltage_v) && _battery_status.voltage_v > 0.f) {
			telemetry.battery_voltage_mv = static_cast<uint16_t>(
							       math::constrain(lroundf(_battery_status.voltage_v * 1000.f), 0L, 65535L));
			power_valid = true;
		}

		if (PX4_ISFINITE(_battery_status.current_a)) {
			telemetry.battery_current_ca = static_cast<int16_t>(
							       math::constrain(lroundf(_battery_status.current_a * 100.f), -32767L, 32767L));
			power_valid = true;
		}

		if (PX4_ISFINITE(_battery_status.remaining) && _battery_status.remaining >= 0.f) {
			telemetry.battery_remaining = static_cast<uint8_t>(
							      math::constrain(lroundf(_battery_status.remaining), 0L, 100L));
		}
	}

	uint8_t rc_count = 0;

	if (_rc_channels.timestamp != 0 && now >= _rc_channels.timestamp
	    && now - _rc_channels.timestamp <= kRcChannelsTimeout && !_rc_channels.signal_lost) {
		rc_count = math::min<uint8_t>(_rc_channels.channel_count, gs_link::kMaxRcChannels);
	}

	telemetry.rc_channel_count = rc_count;
	telemetry.status_flags = buildStatusFlags(now, attitude_valid,
				 motor_found[0] && motor_found[1], power_valid);

	uint8_t payload[gs_link::kMaxPayload] {};
	memcpy(payload, &telemetry, sizeof(telemetry));

	for (uint8_t index = 0; index < rc_count; ++index) {
		const float value = _rc_channels.channels[index];
		int8_t encoded = -128;

		if (PX4_ISFINITE(value)) {
			encoded = static_cast<int8_t>(
					  math::constrain(lroundf(math::constrain(value, -1.f, 1.f) * 127.f),
							  -127L, 127L));
		}

		payload[sizeof(telemetry) + index] = static_cast<uint8_t>(encoded);
	}

	sendPacket(gs_link::PacketClass::Data, static_cast<uint8_t>(gs_link::DataId::Telemetry),
		   0, payload, sizeof(telemetry) + rc_count);
}

void GsLink::sendHeartbeat(hrt_abstime now)
{
	gs_link::HeartbeatPayload heartbeat{};
	heartbeat.uptime_ms = static_cast<uint32_t>(now / 1000);
	heartbeat.session_id = _session_id;
	heartbeat.protocol_version = gs_link::kVersion;
	heartbeat.status_flags = buildStatusFlags(now, false, false, false);
	heartbeat.state = _gate_status.gs_mode_active
			  ? static_cast<uint8_t>(gs_link::LinkState::Auto)
			  : static_cast<uint8_t>(gs_link::LinkState::Ready);
	heartbeat.error_code = _last_error;
	sendPacket(gs_link::PacketClass::Command,
		   static_cast<uint8_t>(gs_link::CommandId::Heartbeat), 0,
		   &heartbeat, sizeof(heartbeat));
}

void GsLink::publishControl(bool valid, hrt_abstime now)
{
	gs_link_control_s control{};
	control.timestamp = now;
	control.timestamp_sample = _last_rc_arrival;
	control.valid = valid && _rc_data_valid;
	control.heartbeat_ok = heartbeatFresh(now);
	control.roll = _rc_roll;
	control.pitch = _rc_pitch;
	control.yaw = _rc_yaw;
	control.throttle = _rc_throttle;
	control.link_quality = _rc_link_quality;
	control.sequence = _rc_sequence;
	control.session_id = _session_id;
	_control_pub.publish(control);
}

bool GsLink::heartbeatFresh(hrt_abstime now) const
{
	return _last_gs_heartbeat != 0 && now >= _last_gs_heartbeat
	       && now - _last_gs_heartbeat <= kHeartbeatTimeout;
}

bool GsLink::rcDataFresh(hrt_abstime now) const
{
	return _rc_data_valid && _last_rc_arrival != 0 && now >= _last_rc_arrival
	       && now - _last_rc_arrival <= kRcTimeout;
}

int GsLink::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("port: %s, baud: %lu, open: %d",
		 _serial.getPort(), (unsigned long)_serial.getBaudrate(),
		 _serial.isOpen() ? 1 : 0);
	PX4_INFO("session: %08lx, rx/tx packets: %lu/%lu",
		 (unsigned long)_session_id, (unsigned long)_rx_packets,
		 (unsigned long)_tx_packets);
	PX4_INFO("crc/length/version errors: %lu/%lu/%lu",
		 (unsigned long)_crc_errors, (unsigned long)_length_errors,
		 (unsigned long)_version_errors);
	PX4_INFO("link: heartbeat %s (%llu ms), RC %s (%llu ms), GS heartbeat session %08lx",
		 heartbeatFresh(now) ? "OK" : "stale",
		 (unsigned long long)(_last_gs_heartbeat == 0 ? 0
				      : hrt_elapsed_time(&_last_gs_heartbeat) / 1000),
		 rcDataFresh(now) ? "OK" : "stale",
		 (unsigned long long)(_last_rc_arrival == 0 ? 0
				      : hrt_elapsed_time(&_last_rc_arrival) / 1000),
		 (unsigned long)_gs_session_id);
	PX4_INFO("last RC: seq %u, quality %u, axes %.3f %.3f %.3f %.3f, channels %u",
		 (unsigned)_rc_sequence, (unsigned)_rc_link_quality,
		 (double)_rc_roll, (double)_rc_pitch, (double)_rc_yaw, (double)_rc_throttle,
		 (unsigned)_rc_channels.channel_count);
	PX4_INFO("unknown commands: %lu, last error: %u, write failures: %lu, last write errno: %d",
		 (unsigned long)_unknown_commands, (unsigned)_last_error,
		 (unsigned long)_write_failures, _last_write_errno);
	PX4_INFO("serial open failures: %lu", (unsigned long)_open_failures);
	return 0;
}

int GsLink::task_spawn(int argc, char *argv[])
{
	const char *device = "/dev/ttyS0";
	uint32_t baudrate = 4000000;

	for (int i = 1; i < argc; ++i) {
		if (!strcmp(argv[i], "-d") && i + 1 < argc) {
			device = argv[++i];

		} else if (!strcmp(argv[i], "-b") && i + 1 < argc) {
			baudrate = static_cast<uint32_t>(strtoul(argv[++i], nullptr, 10));
		}
	}

	GsLink *instance = new GsLink(device, baudrate);

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int GsLink::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GsLink::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GS-Link v1 serial transport for the RC-supervised ground-station link.
This build is communication-only: takeoff, land and abort commands are
acknowledged as unsupported and never change the vehicle state.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gs_link", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gs_link_main(int argc, char *argv[])
{
	return GsLink::main(argc, argv);
}
