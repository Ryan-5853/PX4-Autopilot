/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "ControllerTypes.hpp"

namespace final_motor_output_map
{

// The LQI continues to compute in its pre-map command domain. Both physical
// motor outputs are multiplied by this scale at the actuator-output boundary.
// The generated motor-speed table and LQR motor input model must be calibrated
// in the same pre-map domain; changing this value alone invalidates both.
// The 2026-10-26 calibration uses:
//   controller command x -> calibrated motor speed at physical command 0.5*x.
static constexpr float kDefaultMotorScale{0.5f};

inline void apply(ActuatorSetpoint &output)
{
	for (int motor = 0; motor < ActuatorSetpoint::kMotorCount; ++motor) {
		output.motors[motor] *= kDefaultMotorScale;
	}
}

} // namespace final_motor_output_map
