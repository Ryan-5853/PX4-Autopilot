/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * GS-Link v1 wire contract.
 *
 * All multi-byte fields are little-endian. PX4 ARM targets in this project are
 * little-endian; a big-endian port must serialize fields explicitly instead
 * of using these packed structs directly.
 */
namespace gs_link
{

static constexpr uint8_t kMagic0{0xA5};
static constexpr uint8_t kMagic1{0x5A};
static constexpr uint8_t kVersion{1};
static constexpr size_t kHeaderSize{10};
static constexpr size_t kCrcSize{2};
static constexpr size_t kMaxPayload{512};
static constexpr size_t kMaxPacket{kHeaderSize + kMaxPayload + kCrcSize};
static constexpr uint8_t kMaxRcChannels{18};

enum class PacketClass : uint8_t {
	Data = 0x01,
	Command = 0x02,
};

enum class DataId : uint8_t {
	Rc = 0x01,
	Telemetry = 0x02,
};

enum class CommandId : uint8_t {
	Ping = 0x01,
	Heartbeat = 0x02,
	Takeoff = 0x03,
	Land = 0x04,
	Abort = 0x05,
	Ack = 0x7F,
};

enum Flags : uint8_t {
	AckRequired = 1u << 0,
	Retry = 1u << 1,
	Urgent = 1u << 2,
	Response = 1u << 3,
};

enum RcFlags : uint8_t {
	RcValid = 1u << 0,
	GsReady = 1u << 1,
};

enum StatusFlags : uint16_t {
	Armed = 1u << 0,
	GsModeActive = 1u << 1,
	RcValidStatus = 1u << 2,
	RcSwitchSelected = 1u << 3,
	Failsafe = 1u << 4,
	AttitudeValid = 1u << 5,
	MotorRpmValid = 1u << 6,
	PowerValid = 1u << 7,
	BatteryWarning = 1u << 8,
	CommandBusy = 1u << 9,
	TakeoffActive = 1u << 10,
	LandingActive = 1u << 11,
	LinkOk = 1u << 12,
	RcSwitchValid = 1u << 13,
	RcChannelsValid = 1u << 14,
};

enum class AckResult : uint8_t {
	Ok = 0,
	Unsupported = 1,
	BadLength = 2,
	BadArgument = 3,
	RejectedSafety = 4,
	Busy = 5,
	Timeout = 6,
	Duplicate = 7,
	BadVersion = 8,
	InternalError = 9,
	StaleSession = 10,
	LinkStale = 11,
};

enum class LinkState : uint8_t {
	Boot = 0,
	Ready = 1,
	Auto = 2,
	Failsafe = 3,
	Error = 4,
};

struct __attribute__((packed)) Header {
	uint8_t magic0{kMagic0};
	uint8_t magic1{kMagic1};
	uint8_t version{kVersion};
	uint8_t packet_class{0};
	uint8_t message_id{0};
	uint8_t flags{0};
	uint16_t sequence{0};
	uint16_t payload_len{0};
};

struct __attribute__((packed)) RcPayload {
	uint32_t sender_time_ms{0};
	int16_t roll{0};
	int16_t pitch{0};
	int16_t yaw{0};
	int16_t throttle{0};
	uint8_t flags{0};
	uint8_t link_quality{255};
	uint16_t reserved{0};
};

struct __attribute__((packed)) TelemetryPayload {
	uint32_t timestamp_ms{0};
	uint16_t status_flags{0};
	int16_t roll_cdeg{0};
	int16_t pitch_cdeg{0};
	int16_t yaw_cdeg{0};
	uint16_t motor1_rpm{0xFFFF};
	uint16_t motor2_rpm{0xFFFF};
	uint16_t battery_voltage_mv{0};
	int16_t battery_current_ca{INT16_MIN};
	uint8_t battery_remaining{255};
	uint8_t rc_channel_count{0};
};

struct __attribute__((packed)) PingPayload {
	uint32_t token{0};
};

struct __attribute__((packed)) HeartbeatPayload {
	uint32_t uptime_ms{0};
	uint32_t session_id{0};
	uint16_t status_flags{0};
	uint16_t protocol_version{kVersion};
	uint8_t state{static_cast<uint8_t>(LinkState::Boot)};
	uint8_t error_code{0};
	uint16_t reserved{0};
};

struct __attribute__((packed)) TakeoffPayload {
	int32_t target_altitude_mm{0};
	uint16_t max_climb_cm_s{0};
	uint16_t timeout_ms{0};
	uint32_t command_token{0};
	uint32_t session_id{0};
};

struct __attribute__((packed)) LandPayload {
	uint16_t max_descent_cm_s{0};
	uint16_t timeout_ms{0};
	uint32_t command_token{0};
	uint32_t session_id{0};
};

struct __attribute__((packed)) AbortPayload {
	uint32_t reason_code{0};
	uint32_t command_token{0};
	uint32_t session_id{0};
};

struct __attribute__((packed)) AckPayload {
	uint8_t acked_class{0};
	uint8_t acked_id{0};
	uint16_t acked_seq{0};
	uint8_t result{0};
	uint8_t detail{0};
	uint16_t reserved{0};
	uint32_t echo_token{0};
};

static_assert(sizeof(Header) == kHeaderSize, "GS-Link header size mismatch");
static_assert(sizeof(RcPayload) == 16, "GS-Link RC payload size mismatch");
static_assert(sizeof(TelemetryPayload) == 22, "GS-Link telemetry base size mismatch");
static_assert(sizeof(PingPayload) == 4, "GS-Link ping payload size mismatch");
static_assert(sizeof(HeartbeatPayload) == 16, "GS-Link heartbeat payload size mismatch");
static_assert(sizeof(TakeoffPayload) == 16, "GS-Link takeoff payload size mismatch");
static_assert(sizeof(LandPayload) == 12, "GS-Link land payload size mismatch");
static_assert(sizeof(AbortPayload) == 12, "GS-Link abort payload size mismatch");
static_assert(sizeof(AckPayload) == 12, "GS-Link ACK payload size mismatch");
static_assert(sizeof(TelemetryPayload) + kMaxRcChannels <= kMaxPayload,
	      "GS-Link telemetry exceeds payload limit");

inline uint16_t crc16(const uint8_t *data, size_t length)
{
	uint16_t crc{0xFFFF};

	for (size_t i = 0; i < length; ++i) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;

		for (int bit = 0; bit < 8; ++bit) {
			crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
			      : static_cast<uint16_t>(crc << 1);
		}
	}

	return crc;
}

} // namespace gs_link
