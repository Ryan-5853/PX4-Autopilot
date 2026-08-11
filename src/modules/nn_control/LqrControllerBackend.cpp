/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ControllerBackend.hpp"

#include "LqiControllerCore.hpp"
#include "LqiManualReference.hpp"
#include "LqiNominalModel.hpp"

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

	if (!_lqi_reference.build(input.attitude_q, input.angular_velocity,
				  input.rc_roll, input.rc_pitch, input.rc_yaw,
				  input.rc_throttle, core_input)) {
		return false;
	}

	core_input.motor_rpm[0] = input.motor_rpm[0];
	core_input.motor_rpm[1] = input.motor_rpm[1];
	core_input.motor_rpm_valid = input.motor_rpm_valid;

	LqiControllerCore::ControlOutput control {};

	if (!_lqi.step(core_input, control)) {
		return false;
	}

	// Deployment ownership boundary: upper is the pilot command unchanged;
	// nominal LQI owns lower plus the three servos.
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
	_lqi_reference.reset();
}

const char *ControllerBackend::name() const
{
	return "lqi_nominal";
}

const char *ControllerBackend::mode() const
{
	return "external upper + nominal 4-output LQI";
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
	_lqi_reference.reset();
	return true;
}

bool ControllerBackend::verificationStep(
	const float (&features)[kVerificationInputSize],
	float (&output)[kVerificationOutputSize])
{
	LqiControllerCore::Input core_input {};

	if (!_lqi_reference.build(features, &features[4], features[16], features[17],
				  features[18], features[19], core_input)) {
		return false;
	}

	core_input.motor_rpm_valid = false;

	LqiControllerCore::ControlOutput control {};

	if (!_lqi.step(core_input, control)) {
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
	return lqi_nominal::kModel.checksum;
}

uint32_t ControllerBackend::verificationWeightBytes() const
{
	return lqi_nominal::kModel.weight_bytes;
}

uint32_t ControllerBackend::verificationMacsPerInference() const
{
	return LqiControllerModel::kOutputSize * LqiControllerModel::kStateSize;
}
