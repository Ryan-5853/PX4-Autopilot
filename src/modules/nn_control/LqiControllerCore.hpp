/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "LqiWeights.hpp"

#include <cmath>

/**
 * Dependency-free Plan A 4-output LQI core.
 *
 * Contract:
 * - The upper rotor is owned externally (RC throttle in the PX4 framework).
 * - The LQI owns the lower rotor and the three servos (4 outputs).
 * - State (13): [roll_error, pitch_error, p, q, r, upper_f, lower_f,
 *   servo_1, servo_2, servo_3, integral_roll, integral_pitch,
 *   integral_yaw_rate].
 * - All five actuator states are command-driven first-order observers; no
 *   ESC telemetry or servo-angle feedback is used.
 * - Fixed 500 Hz step (dt = 2 ms) matching the gain discretization.
 *
 * The class is host-compilable (no PX4 dependencies) so the same math is
 * exercised by the host golden-vector check and the firmware backend.
 */
class LqiControllerCore
{
public:
	struct Input {
		float attitude_q[4];	   // [w, x, y, z], body -> world (Hamilton)
		float angular_velocity[3]; // rad/s, body FRD
		float rc_throttle;	   // [-1, 1]
		float rc_yaw;		   // [-1, 1]
	};

	struct ControlOutput {
		float upper;   // external upper rotor command [0, 1]
		float lower;   // LQI lower rotor command [0, 1]
		float servos[3]; // LQI servo commands [-1, 1]
	};

	static constexpr int kPersistentSize = lqi_weights::kPersistentSize;

	LqiControllerCore() = default;

	void reset();

	/**
	 * One 500 Hz step from the persistent state held by this instance.
	 * Returns false when the computed output is non-finite.
	 */
	bool step(const Input &input, ControlOutput &output);

	/**
	 * Pure single-step evaluation used by the verification protocol and the
	 * host golden-vector check. `persistent` is [actuator(5), integral(3)].
	 */
	static bool stepFromState(const Input &input,
				  const float persistent[kPersistentSize],
				  ControlOutput &output,
				  float next_persistent[kPersistentSize]);

	const float *persistent() const { return _persistent; }
	const ControlOutput &lastOutput() const { return _last_output; }

private:
	float _persistent[kPersistentSize]{};
	ControlOutput _last_output{};
};

inline void LqiControllerCore::reset()
{
	for (float &value : _persistent) {
		value = 0.0f;
	}

	_last_output = {};
}

inline bool LqiControllerCore::stepFromState(
	const Input &input,
	const float persistent[kPersistentSize],
	ControlOutput &output,
	float next_persistent[kPersistentSize])
{
	using lqi_weights::kDecay;
	using lqi_weights::kGain;
	using lqi_weights::kIntegralLimit;
	using lqi_weights::kLowerTrim;
	using lqi_weights::kSlopeEff;
	using lqi_weights::kUpperTrim;

	// Rotation error from level target. With the identity target the error
	// quaternion is the current attitude, canonicalized so q and -q agree.
	float w = input.attitude_q[0];
	float vx = input.attitude_q[1];
	float vy = input.attitude_q[2];
	float vz = input.attitude_q[3];

	if (w < 0.0f) {
		w = -w;
		vx = -vx;
		vy = -vy;
		vz = -vz;
	}

	const float vnorm = std::sqrt(vx * vx + vy * vy + vz * vz);
	const float vnorm_clamped = (vnorm < 1e-12f) ? 1e-12f : vnorm;
	const float angle = 2.0f * std::atan2(vnorm, (w < 1e-12f) ? 1e-12f : w);
	const float scale = angle / vnorm_clamped;
	const float roll_error = vx * scale;
	const float pitch_error = vy * scale;

	// Yaw tracks rate only (no yaw angle feedback), commanded by the RC yaw
	// stick; roll/pitch track the level reference with zero target rate.
	const float yaw_rate_target = input.rc_yaw * lqi_weights::kYawRateScaleRadS;
	const float p_error = input.angular_velocity[0];
	const float q_error = input.angular_velocity[1];
	const float r_error = input.angular_velocity[2] - yaw_rate_target;

	const float x[lqi_weights::kStateSize] {
		roll_error,
		pitch_error,
		p_error,
		q_error,
		r_error,
		persistent[0],
		persistent[1],
		persistent[2],
		persistent[3],
		persistent[4],
		persistent[5],
		persistent[6],
		persistent[7],
	};

	float delta[lqi_weights::kOutputSize] {};

	for (int row = 0; row < lqi_weights::kOutputSize; ++row) {
		float accumulator = 0.0f;

		for (int col = 0; col < lqi_weights::kStateSize; ++col) {
			accumulator += kGain[row * lqi_weights::kStateSize + col] * x[col];
		}

		delta[row] = -accumulator;
	}

	const float lower_raw = kLowerTrim + delta[0];
	const float servo_raw[3] { delta[1], delta[2], delta[3] };
	const float lower = (lower_raw < 0.0f) ? 0.0f : ((lower_raw > 1.0f) ? 1.0f : lower_raw);

	float servos[3] {};

	for (int index = 0; index < 3; ++index) {
		servos[index] = (servo_raw[index] < -1.0f) ? -1.0f
				: ((servo_raw[index] > 1.0f) ? 1.0f : servo_raw[index]);
	}

	const bool saturated = (lower_raw < 0.0f)
			       || (lower_raw > 1.0f)
			       || (servo_raw[0] < -1.0f) || (servo_raw[0] > 1.0f)
			       || (servo_raw[1] < -1.0f) || (servo_raw[1] > 1.0f)
			       || (servo_raw[2] < -1.0f) || (servo_raw[2] > 1.0f);

	// Integral states: [roll_error, pitch_error, yaw_rate_error], frozen
	// on command saturation to prevent integral windup.
	const float errors[3] { roll_error, pitch_error, r_error };
	float integral_next[3] {};

	for (int axis = 0; axis < 3; ++axis) {
		const float candidate = persistent[5 + axis] + errors[axis] * lqi_weights::kDt;
		const float clamped = (candidate < -kIntegralLimit[axis]) ? -kIntegralLimit[axis]
				      : ((candidate > kIntegralLimit[axis]) ? kIntegralLimit[axis] : candidate);
		integral_next[axis] = saturated ? persistent[5 + axis] : clamped;
	}

	// External upper command (pilot/RC), normalized to [0, 1].
	const float throttle = (input.rc_throttle < -1.0f) ? -1.0f
			       : ((input.rc_throttle > 1.0f) ? 1.0f : input.rc_throttle);
	const float upper = (throttle + 1.0f) * 0.5f;

	// Command-driven actuator observers: state += decay*state + slope_eff*u.
	const float command_deviation[5] {
		upper - kUpperTrim,
		lower - kLowerTrim,
		servos[0],
		servos[1],
		servos[2],
	};

	for (int actuator = 0; actuator < 5; ++actuator) {
		next_persistent[actuator] =
			kDecay[actuator] * persistent[actuator]
			+ kSlopeEff[actuator] * command_deviation[actuator];
	}

	for (int axis = 0; axis < 3; ++axis) {
		next_persistent[5 + axis] = integral_next[axis];
	}

	output.upper = upper;
	output.lower = lower;
	output.servos[0] = servos[0];
	output.servos[1] = servos[1];
	output.servos[2] = servos[2];

	for (float value : output.servos) {
		if (!std::isfinite(value)) {
			return false;
		}
	}

	return std::isfinite(output.upper) && std::isfinite(output.lower);
}

inline bool LqiControllerCore::step(const Input &input, ControlOutput &output)
{
	float next_persistent[kPersistentSize] {};

	if (!stepFromState(input, _persistent, output, next_persistent)) {
		return false;
	}

	for (int index = 0; index < kPersistentSize; ++index) {
		_persistent[index] = next_persistent[index];
	}

	_last_output = output;
	return true;
}
