/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ControllerBackend.hpp"

#include <mathlib/math/Functions.hpp>

bool ControllerBackend::init()
{
	return _controller_network.init()
	       && _verification_network.init()
	       && _verification_network.weightChecksum() == _controller_network.weightChecksum();
}

bool ControllerBackend::run(const ControllerInput &input, ActuatorSetpoint &output)
{
	NeuralControlOutput model_output{};

	if (!_controller_network.run(input, model_output)) {
		return false;
	}

	// The deployed four-output contract is lower motor plus three servos. The
	// upper motor remains under the external collective/throttle command.
	output.motors[0] = math::constrain((input.rc_throttle + 1.f) * 0.5f, 0.f, 1.f);
	output.motors[1] = math::constrain((model_output.controls[0] + 1.f) * 0.5f, 0.f, 1.f);
	output.servos[0] = model_output.controls[1];
	output.servos[1] = model_output.controls[2];
	output.servos[2] = model_output.controls[3];
	return true;
}

void ControllerBackend::reset()
{
	_controller_network.reset();
	_verification_network.reset();
}

const char *ControllerBackend::name() const
{
	return "neural";
}

const char *ControllerBackend::mode() const
{
	return "21-128-128-4 int8 GRU (random-weight dry-run)";
}

const char *ControllerBackend::airframeId() const
{
	return "not-applicable";
}

const char *ControllerBackend::identificationArtifactSha256() const
{
	return "";
}

int32_t ControllerBackend::configuredAirframeNumber() const
{
	return 0;
}

int32_t ControllerBackend::expectedAirframeNumber() const
{
	return 0;
}

bool ControllerBackend::airframeMismatch() const
{
	return false;
}

bool ControllerBackend::actuatorOutputAllowed() const
{
	// This must only become true when generated production weights and their
	// matching quantization metadata replace the deterministic random scaffold.
	return false;
}

bool ControllerBackend::verificationSupported() const
{
	return true;
}

bool ControllerBackend::verificationReset()
{
	_verification_network.reset();
	return true;
}

bool ControllerBackend::verificationStep(
	const float (&features)[kVerificationInputSize],
	float (&output)[kVerificationOutputSize])
{
	static_assert(kVerificationInputSize == NeuralNetworkInterface::kInputSize,
		      "verification input size mismatch");
	static_assert(kVerificationOutputSize == NeuralNetworkInterface::kOutputSize,
		      "verification output size mismatch");
	NeuralControlOutput model_output{};

	if (!_verification_network.runFeatures(features, model_output)) {
		return false;
	}

	for (int i = 0; i < kVerificationOutputSize; ++i) {
		output[i] = model_output.controls[i];
	}

	return true;
}

uint32_t ControllerBackend::verificationChecksum() const
{
	return _verification_network.weightChecksum();
}

uint32_t ControllerBackend::verificationWeightBytes() const
{
	return NeuralNetworkInterface::weightBytes();
}

uint32_t ControllerBackend::verificationMacsPerInference() const
{
	return NeuralNetworkInterface::kMacsPerInference;
}
