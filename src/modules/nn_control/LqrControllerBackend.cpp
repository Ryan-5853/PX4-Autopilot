/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ControllerBackend.hpp"

#include "LqiControllerCore.hpp"

#include <px4_platform_common/defines.h>
#include <parameters/param.h>

#include <cstring>

namespace
{

param_t g_output_enable_handle{PARAM_INVALID};

} // namespace

bool ControllerBackend::init()
{
	g_output_enable_handle = param_find("NN_LQI_OUTPUT_EN");
	return g_output_enable_handle != PARAM_INVALID;
}

bool ControllerBackend::run(const ControllerInput &input, ActuatorSetpoint &output)
{
	LqiControllerCore::Input core_input {};
	memcpy(core_input.attitude_q, input.attitude_q, sizeof(core_input.attitude_q));
	memcpy(core_input.angular_velocity, input.angular_velocity,
	       sizeof(core_input.angular_velocity));
	core_input.rc_throttle = input.rc_throttle;
	core_input.rc_yaw = input.rc_yaw;

	LqiControllerCore::ControlOutput control {};

	if (!_lqi.step(core_input, control)) {
		return false;
	}

	// Plan A ownership boundary: the upper rotor is the external pilot/RC
	// collective; the LQI owns the lower rotor and the three servos.
	output.motors[0] = control.upper;
	output.motors[1] = control.lower;
	output.servos[0] = control.servos[0];
	output.servos[1] = control.servos[1];
	output.servos[2] = control.servos[2];
	return true;
}

void ControllerBackend::reset()
{
	_lqi.reset();
}

const char *ControllerBackend::name() const
{
	return "lqr";
}

const char *ControllerBackend::mode() const
{
	return "Plan A external-collective 4-output LQI";
}

bool ControllerBackend::actuatorOutputAllowed() const
{
	int32_t enabled = 0;

	if (param_get(g_output_enable_handle, &enabled) != PX4_OK) {
		return false;
	}

	// NN_LQI_OUTPUT_EN must stay 0 until bench/HIL validation is complete.
	return enabled != 0;
}

bool ControllerBackend::verificationSupported() const
{
	return true;
}

bool ControllerBackend::verificationReset()
{
	_lqi.reset();
	return true;
}

bool ControllerBackend::verificationStep(
	const float (&features)[kVerificationInputSize],
	float (&output)[kVerificationOutputSize])
{
	LqiControllerCore::Input core_input {};

	for (int index = 0; index < 4; ++index) {
		core_input.attitude_q[index] = features[index];
	}

	for (int index = 0; index < 3; ++index) {
		core_input.angular_velocity[index] = features[4 + index];
	}

	core_input.rc_throttle = features[7];
	core_input.rc_yaw = features[8];

	float persistent[LqiControllerCore::kPersistentSize] {};
	float next_persistent[LqiControllerCore::kPersistentSize] {};

	for (int index = 0; index < LqiControllerCore::kPersistentSize; ++index) {
		persistent[index] = features[9 + index];
	}

	LqiControllerCore::ControlOutput control {};

	if (!LqiControllerCore::stepFromState(
		    core_input, persistent, control, next_persistent)) {
		return false;
	}

	output[0] = control.lower;
	output[1] = control.servos[0];
	output[2] = control.servos[1];
	output[3] = control.servos[2];

	for (float value : output) {
		if (!std::isfinite(value)) {
			return false;
		}
	}

	return true;
}

uint32_t ControllerBackend::verificationChecksum() const
{
	return lqi_weights::kChecksum;
}

uint32_t ControllerBackend::verificationWeightBytes() const
{
	return lqi_weights::kWeightBytes;
}

uint32_t ControllerBackend::verificationMacsPerInference() const
{
	return lqi_weights::kOutputSize * lqi_weights::kStateSize;
}
