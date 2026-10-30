/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ControllerBackend.hpp"

#include "LqiCompositeCore.hpp"
#include "LqiControllerCore.hpp"
#include "LqiManualReference.hpp"
#include "LqiNominalModel.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <parameters/param.h>

#include <cstring>

namespace
{

param_t g_output_enable_handle{PARAM_INVALID};
param_t g_model_index_handle{PARAM_INVALID};
param_t g_airframe_id_handle{PARAM_INVALID};

} // namespace

bool ControllerBackend::init()
{
	g_output_enable_handle = param_find("NN_LQI_OUTPUT_EN");
	g_model_index_handle = param_find("NN_LQI_MODEL");
	g_airframe_id_handle = param_find("NN_AIRFRAME_ID");

	if (g_output_enable_handle == PARAM_INVALID || g_model_index_handle == PARAM_INVALID
	    || g_airframe_id_handle == PARAM_INVALID) {
		return false;
	}

	int32_t model_index = 0;

	if (param_get(g_model_index_handle, &model_index) != PX4_OK) {
		return false;
	}

	// 0 = nominal 13-state, 1 = external-upper identified 19-state,
	// 2 = coaxial pilot-acceleration common/differential + servo LQI.
	if (model_index < 0 || model_index > 2) {
		return false;
	}

	_model_index = static_cast<int>(model_index);

	if (_model_index != 0 && configuredAirframeNumber() != expectedAirframeNumber()) {
		PX4_WARN("identified LQI airframe mismatch: model=%s expected=%ld configured=%ld",
			 (_model_index == 2)
			 ? lqi_coaxial_collective::kAirframeId : lqi_identified::kAirframeId,
			 (long)expectedAirframeNumber(),
			 (long)configuredAirframeNumber());
	}

	return true;
}

bool ControllerBackend::run(const ControllerInput &input, ActuatorSetpoint &output)
{
	LqiControllerCore::Input core_input {};

	bool built = false;

	if (_model_index == 2) {
		built = _lqi_coaxial_collective_reference.build(
				input.attitude_q, input.angular_velocity,
				input.rc_roll, input.rc_pitch, input.rc_yaw,
				input.rc_throttle, core_input);

	} else if (_model_index == 1) {
		built = _lqi_identified_reference.build(
				input.attitude_q, input.angular_velocity,
				input.rc_roll, input.rc_pitch, input.rc_yaw,
				input.rc_throttle, core_input);

	} else {
		built = _lqi_nominal_reference.build(
				input.attitude_q, input.angular_velocity,
				input.rc_roll, input.rc_pitch, input.rc_yaw,
				input.rc_throttle, core_input);
	}

	if (!built) {
		return false;
	}

	core_input.motor_rpm[0] = input.motor_rpm[0];
	core_input.motor_rpm[1] = input.motor_rpm[1];
	core_input.motor_rpm_valid = input.motor_rpm_valid;

	for (int index = 0; index < 3; ++index) {
		core_input.acceleration_b[index] = input.acceleration[index];
	}

	LqiControllerCore::ControlOutput control {};
	bool stepped = false;

	if (_model_index == 2) {
		stepped = _lqi_coaxial_collective.step(core_input, control);

	} else if (_model_index == 1) {
		stepped = _lqi_identified.step(core_input, control);

	} else {
		stepped = _lqi_nominal.step(core_input, control);
	}

	if (!stepped) {
		return false;
	}

	// Deployment ownership boundary: upper is the pilot command unchanged;
	// the LQI owns lower plus the three servos.
	output.motors[0] = control.upper;
	output.motors[1] = control.lower;
	output.servos[0] = control.servos[0];
	output.servos[1] = control.servos[1];
	output.servos[2] = control.servos[2];
	return true;
}

void ControllerBackend::reset()
{
	_lqi_nominal.reset();
	_lqi_identified.reset();
	_lqi_coaxial_collective.reset();
	_lqi_nominal_reference.reset();
	_lqi_identified_reference.reset();
	_lqi_coaxial_collective_reference.reset();
}

const char *ControllerBackend::name() const
{
	if (_model_index == 2) {
		return "lqi_coaxial_collective";
	}

	return (_model_index == 1) ? "lqi_identified" : "lqi_nominal";
}

const char *ControllerBackend::mode() const
{
	if (_model_index == 2) {
		return "pilot acceleration + dual-motor common/differential + servo roll/pitch LQI";
	}

	return (_model_index == 1)
	       ? "external upper + airframe-bound identified 19-state composite LQI"
	       : "external upper + nominal 4-output LQI";
}

const char *ControllerBackend::airframeId() const
{
	if (_model_index == 2) {
		return lqi_coaxial_collective::kAirframeId;
	}

	return (_model_index == 1) ? lqi_identified::kAirframeId : "nominal-generic";
}

const char *ControllerBackend::identificationArtifactSha256() const
{
	if (_model_index == 2) {
		return lqi_coaxial_collective::kIdentificationArtifactSha256;
	}

	return (_model_index == 1) ? lqi_identified::kIdentificationArtifactSha256 : "";
}

int32_t ControllerBackend::configuredAirframeNumber() const
{
	int32_t value{0};

	if (g_airframe_id_handle == PARAM_INVALID
	    || param_get(g_airframe_id_handle, &value) != PX4_OK) {
		return -1;
	}

	return value;
}

int32_t ControllerBackend::expectedAirframeNumber() const
{
	if (_model_index == 2) {
		return static_cast<int32_t>(lqi_coaxial_collective::kAirframeNumber);
	}

	return (_model_index == 1)
	       ? static_cast<int32_t>(lqi_identified::kAirframeNumber)
	       : 0;
}

bool ControllerBackend::airframeMismatch() const
{
	return _model_index != 0
	       && configuredAirframeNumber() != expectedAirframeNumber();
}

bool ControllerBackend::actuatorOutputAllowed() const
{
	if (airframeMismatch()) {
		return false;
	}

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
	_lqi_nominal.reset();
	_lqi_identified.reset();
	_lqi_coaxial_collective.reset();
	_lqi_nominal_reference.reset();
	_lqi_identified_reference.reset();
	_lqi_coaxial_collective_reference.reset();
	return true;
}

bool ControllerBackend::verificationStep(
	const float (&features)[kVerificationInputSize],
	float (&output)[kVerificationOutputSize])
{
	LqiControllerCore::Input core_input {};

	bool built = false;

	if (_model_index == 2) {
		built = _lqi_coaxial_collective_reference.build(
				features, &features[4], features[16],
				features[17], features[18], features[19], core_input);

	} else if (_model_index == 1) {
		built = _lqi_identified_reference.build(
				features, &features[4], features[16],
				features[17], features[18], features[19], core_input);

	} else {
		built = _lqi_nominal_reference.build(
				features, &features[4], features[16],
				features[17], features[18], features[19], core_input);
	}

	if (!built) {
		return false;
	}

	core_input.motor_rpm_valid = false;

	LqiControllerCore::ControlOutput control {};
	bool stepped = false;

	if (_model_index == 2) {
		stepped = _lqi_coaxial_collective.step(core_input, control);

	} else if (_model_index == 1) {
		stepped = _lqi_identified.step(core_input, control);

	} else {
		stepped = _lqi_nominal.step(core_input, control);
	}

	if (!stepped) {
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
	if (_model_index == 2) {
		return lqi_coaxial_collective::kModel.checksum;
	}

	return (_model_index == 1) ? lqi_identified::kModel.checksum
	       : lqi_nominal::kModel.checksum;
}

uint32_t ControllerBackend::verificationWeightBytes() const
{
	if (_model_index == 2) {
		return lqi_coaxial_collective::kModel.weight_bytes;
	}

	return (_model_index == 1) ? lqi_identified::kModel.weight_bytes
	       : lqi_nominal::kModel.weight_bytes;
}

uint32_t ControllerBackend::verificationMacsPerInference() const
{
	if (_model_index == 2) {
		return LqiCoaxialCollectiveModel::kAttitudeInputSize
		       * LqiCoaxialCollectiveModel::kStateSize;
	}

	return (_model_index == 1)
	       ? LqiCompositeControllerModel::kOutputSize * LqiCompositeControllerModel::kStateSize
	       : LqiControllerModel::kOutputSize * LqiControllerModel::kStateSize;
}
