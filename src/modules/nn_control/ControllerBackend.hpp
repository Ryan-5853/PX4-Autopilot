/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "ControllerTypes.hpp"

#include <px4_platform_common/px4_config.h>

#include <stdint.h>

#if defined(CONFIG_NN_CONTROL_BACKEND_NEURAL)
# include "NeuralNetworkInterface.hpp"
#endif

/**
 * Compile-time-selected controller backend.
 *
 * Each backend source implements this exact API. Optional neural verification
 * methods return unsupported for backends that do not use that protocol.
 */
class ControllerBackend
{
public:
	static constexpr int kVerificationInputSize{21};
	static constexpr int kVerificationOutputSize{4};

	bool init();
	bool run(const ControllerInput &input, ActuatorSetpoint &output);
	void reset();

	const char *name() const;
	const char *mode() const;
	bool actuatorOutputAllowed() const;

	bool verificationSupported() const;
	bool verificationReset();
	bool verificationStep(const float (&features)[kVerificationInputSize],
			      float (&output)[kVerificationOutputSize]);
	uint32_t verificationChecksum() const;
	uint32_t verificationWeightBytes() const;
	uint32_t verificationMacsPerInference() const;

private:
#if defined(CONFIG_NN_CONTROL_BACKEND_NEURAL)
	NeuralNetworkInterface _controller_network{};
	NeuralNetworkInterface _verification_network{};
#endif
};
