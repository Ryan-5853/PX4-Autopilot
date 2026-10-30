/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ControllerBackend.hpp"

bool ControllerBackend::init()
{
	return true;
}

bool ControllerBackend::run(const ControllerInput &, ActuatorSetpoint &)
{
	// Replace this body with the flight-validated hybrid implementation.
	// Returning false keeps every actuator inhibited until then.
	return false;
}

void ControllerBackend::reset()
{
}

const char *ControllerBackend::name() const
{
	return "hybrid";
}

const char *ControllerBackend::mode() const
{
	return "implementation scaffold";
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
	return false;
}

bool ControllerBackend::verificationSupported() const
{
	return false;
}

bool ControllerBackend::verificationReset()
{
	return false;
}

bool ControllerBackend::verificationStep(const float (&)[kVerificationInputSize],
		float (&)[kVerificationOutputSize])
{
	return false;
}

uint32_t ControllerBackend::verificationChecksum() const
{
	return 0;
}

uint32_t ControllerBackend::verificationWeightBytes() const
{
	return 0;
}

uint32_t ControllerBackend::verificationMacsPerInference() const
{
	return 0;
}
