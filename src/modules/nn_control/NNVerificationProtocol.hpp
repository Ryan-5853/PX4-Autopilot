/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

namespace nn_verification
{

static constexpr uint16_t kRequestId{42420};
static constexpr uint16_t kResponseId{42421};
static constexpr char kRequestName[]{"NNGRU_REQ"};
static constexpr char kResponseName[]{"NNGRU_RSP"};
static constexpr uint8_t kProtocolVersion{1};

enum class Command : uint8_t {
	Info = 0,
	Reset = 1,
	Step = 2,
};

enum class Status : uint8_t {
	Ok = 0,
	BadVersion = 1,
	BadChecksum = 2,
	InvalidObservation = 3,
	InferenceFailed = 4,
	UnknownCommand = 5,
};

namespace request
{
static constexpr int kVersion{0};
static constexpr int kCommand{1};
static constexpr int kSequence{2};
static constexpr int kChecksumLow{3};
static constexpr int kChecksumHigh{4};
static constexpr int kObservationFirst{5};
}

namespace response
{
static constexpr int kVersion{0};
static constexpr int kStatus{1};
static constexpr int kSequence{2};
static constexpr int kInferenceTimeUs{3};
static constexpr int kOutputFirst{4};
static constexpr int kChecksumLow{8};
static constexpr int kChecksumHigh{9};
static constexpr int kWeightBytes{10};
static constexpr int kMacsPerInference{11};
}

inline uint32_t checksumFromFloatWords(float low, float high)
{
	return static_cast<uint32_t>(low) | (static_cast<uint32_t>(high) << 16);
}

inline float checksumLow(uint32_t checksum)
{
	return static_cast<float>(checksum & 0xffffu);
}

inline float checksumHigh(uint32_t checksum)
{
	return static_cast<float>((checksum >> 16) & 0xffffu);
}

} // namespace nn_verification
