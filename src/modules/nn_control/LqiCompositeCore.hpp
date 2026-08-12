/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "LqiCompositeControllerModel.hpp"
#include "LqiControllerCore.hpp"

#include <cmath>

/**
 * 19-state composite LQI core (real-log identified deployment variant).
 *
 * State (19): [roll_error, pitch_error, p, q, r, upper_f, lower_f,
 *              latent(3 basis x 3 axes), integral_roll, integral_pitch,
 *              integral_yaw_rate].
 *
 * Persistent (14): [upper_f, lower_f, latent(9), integral(3)].  The motor
 * states use fresh ESC RPM when available and otherwise command-driven
 * first-order observers; the latent states are command-driven basis filters
 * whose input matrix is (mode_transform @ diag(servo_slopes)).
 *
 * The math mirrors ``flight_identification.sim2real_composite`` and the
 * generator's Python mirror; host golden vectors pin both sides together.
 */
class LqiCompositeCore
{
public:
	using Input = LqiControllerCore::Input;
	using ControlOutput = LqiControllerCore::ControlOutput;

	static constexpr int kPersistentSize = LqiCompositeControllerModel::kPersistentSize;
	static constexpr float kIdleCollectiveThreshold{0.0f};
	static constexpr float kMaxUpperLowerDelta{0.2f};

	explicit LqiCompositeCore(const LqiCompositeControllerModel &model)
		: _model(model) {}

	void reset()
	{
		for (float &value : _persistent) {
			value = 0.0f;
		}

		_last_output = {};
	}

	bool step(const Input &input, ControlOutput &output)
	{
		float next_persistent[kPersistentSize] {};

		if (!stepFromState(_model, input, _persistent, output, next_persistent)) {
			return false;
		}

		for (int index = 0; index < kPersistentSize; ++index) {
			_persistent[index] = next_persistent[index];
		}

		_last_output = output;
		return true;
	}

	static bool stepFromState(const LqiCompositeControllerModel &model,
				  const Input &input,
				  const float persistent[kPersistentSize],
				  ControlOutput &output,
				  float next_persistent[kPersistentSize]);

	const float *persistent() const { return _persistent; }
	const ControlOutput &lastOutput() const { return _last_output; }

private:
	const LqiCompositeControllerModel &_model;
	float _persistent[kPersistentSize] {};
	ControlOutput _last_output{};
};

inline bool LqiCompositeCore::stepFromState(
	const LqiCompositeControllerModel &model,
	const Input &input,
	const float persistent[kPersistentSize],
	ControlOutput &output,
	float next_persistent[kPersistentSize])
{
	if (model.schema_version != LqiCompositeControllerModel::kSchemaVersion
	    || model.world_frame != LqiCompositeControllerModel::WorldFrame::NED
	    || model.body_frame != LqiCompositeControllerModel::BodyFrame::FRD
	    || model.quaternion_convention
	       != LqiCompositeControllerModel::QuaternionConvention::HamiltonWxyzBodyToWorld
	    || model.collective_contract
	       != LqiCompositeControllerModel::CollectiveContract::ExternalUpperLqiLowerAndServos
	    || model.actuator_observer != LqiCompositeControllerModel::ActuatorObserver::MotorRpmOrCommandDriven
	    || model.gain == nullptr || model.motor_decay == nullptr
	    || model.motor_slope_eff == nullptr || model.latent_decay == nullptr
	    || model.latent_input == nullptr || model.integral_limit == nullptr
	    || model.motor_command_to_rad_s == nullptr || model.motor_table_points < 2
	    || !(model.dt_s > 0.0f)) {
		return false;
	}

	// q_error = conjugate(q_target_wb) * q_current_wb (target-to-current),
	// identical to LqiControllerCore and flight_controller.math.
	const float tw = input.target_attitude_q_wb[0];
	const float tx = -input.target_attitude_q_wb[1];
	const float ty = -input.target_attitude_q_wb[2];
	const float tz = -input.target_attitude_q_wb[3];
	const float cw = input.attitude_q_wb[0];
	const float cx = input.attitude_q_wb[1];
	const float cy = input.attitude_q_wb[2];
	const float cz = input.attitude_q_wb[3];
	float w = tw * cw - tx * cx - ty * cy - tz * cz;
	float vx = tw * cx + tx * cw + ty * cz - tz * cy;
	float vy = tw * cy - tx * cz + ty * cw + tz * cx;
	float vz = tw * cz + tx * cy - ty * cx + tz * cw;

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

	const float p_error = input.angular_velocity_b[0] - input.target_angular_velocity_b[0];
	const float q_error = input.angular_velocity_b[1] - input.target_angular_velocity_b[1];
	const float r_error = input.angular_velocity_b[2] - input.target_angular_velocity_b[2];

	const float collective = (input.collective_base < 0.0f) ? 0.0f
				 : ((input.collective_base > 1.0f) ? 1.0f : input.collective_base);

	if (collective <= kIdleCollectiveThreshold) {
		output.upper = 0.0f;
		output.lower = 0.0f;
		output.servos[0] = 0.0f;
		output.servos[1] = 0.0f;
		output.servos[2] = 0.0f;

		for (int index = 0; index < kPersistentSize; ++index) {
			next_persistent[index] = persistent[index];
		}

		return true;
	}

	// Piecewise-linear motor target lookup, shared by both coaxial motors.
	int upper_point = 1;

	while (upper_point < model.motor_table_points - 1
	       && collective > model.motor_command_to_rad_s[2 * upper_point]) {
		++upper_point;
	}

	const int lower_point = upper_point - 1;
	const float command_0 = model.motor_command_to_rad_s[2 * lower_point];
	const float speed_0 = model.motor_command_to_rad_s[2 * lower_point + 1];
	const float command_1 = model.motor_command_to_rad_s[2 * upper_point];
	const float speed_1 = model.motor_command_to_rad_s[2 * upper_point + 1];
	const float command_clamped = (collective < model.motor_command_to_rad_s[0])
				      ? model.motor_command_to_rad_s[0]
				      : ((collective > model.motor_command_to_rad_s[2 * (model.motor_table_points - 1)])
					 ? model.motor_command_to_rad_s[2 * (model.motor_table_points - 1)] : collective);
	const float target_speed = speed_0 + (command_clamped - command_0) * (speed_1 - speed_0)
				   / (command_1 - command_0);
	const float target_upper_dev = target_speed - model.upper_trim_rad_s;
	const float target_lower_dev = target_speed - model.lower_trim_rad_s;
	const float upper_motor_error = input.motor_rpm_valid
		? std::fabs(input.motor_rpm[0]) * model.rpm_to_rad_s - model.upper_trim_rad_s - target_upper_dev
		: persistent[0] - target_upper_dev;
	const float lower_motor_error = input.motor_rpm_valid
		? std::fabs(input.motor_rpm[1]) * model.rpm_to_rad_s - model.lower_trim_rad_s - target_lower_dev
		: persistent[1] - target_lower_dev;

	const float x[LqiCompositeControllerModel::kStateSize] {
		roll_error,
		pitch_error,
		p_error,
		q_error,
		r_error,
		upper_motor_error,
		lower_motor_error,
		persistent[2],
		persistent[3],
		persistent[4],
		persistent[5],
		persistent[6],
		persistent[7],
		persistent[8],
		persistent[9],
		persistent[10],
		persistent[11],
		persistent[12],
		persistent[13],
	};

	float delta[LqiCompositeControllerModel::kOutputSize] {};

	for (int row = 0; row < LqiCompositeControllerModel::kOutputSize; ++row) {
		float accumulator = 0.0f;

		for (int col = 0; col < LqiCompositeControllerModel::kStateSize; ++col) {
			accumulator += model.gain[row * LqiCompositeControllerModel::kStateSize + col] * x[col];
		}

		delta[row] = -accumulator;
	}

	const float lower_raw = collective + delta[0];
	const float servo_raw[3] { delta[1], delta[2], delta[3] };
	const float upper = collective;
	const float lower_min = (upper - kMaxUpperLowerDelta < 0.0f)
				? 0.0f : (upper - kMaxUpperLowerDelta);
	const float lower_max = (upper + kMaxUpperLowerDelta > 1.0f)
				? 1.0f : (upper + kMaxUpperLowerDelta);
	const float lower = (lower_raw < lower_min) ? lower_min
			    : ((lower_raw > lower_max) ? lower_max : lower_raw);

	float servos[3] {};

	for (int index = 0; index < 3; ++index) {
		servos[index] = (servo_raw[index] < -1.0f) ? -1.0f
				: ((servo_raw[index] > 1.0f) ? 1.0f : servo_raw[index]);
	}

	const bool saturated = (lower_raw < lower_min)
			       || (lower_raw > lower_max)
			       || (servo_raw[0] < -1.0f) || (servo_raw[0] > 1.0f)
			       || (servo_raw[1] < -1.0f) || (servo_raw[1] > 1.0f)
			       || (servo_raw[2] < -1.0f) || (servo_raw[2] > 1.0f);

	// Integral states frozen on command saturation to prevent windup.
	const float errors[3] { roll_error, pitch_error, r_error };
	float integral_next[3] {};

	for (int axis = 0; axis < 3; ++axis) {
		const float candidate = persistent[11 + axis] + errors[axis] * model.dt_s;
		const float clamped = (candidate < -model.integral_limit[axis])
				      ? -model.integral_limit[axis]
				      : ((candidate > model.integral_limit[axis]) ? model.integral_limit[axis] : candidate);
		integral_next[axis] = saturated ? persistent[11 + axis] : clamped;
	}

	// Command-driven motor observers (deviations from trim).
	const float motor_command_dev[2] {
		upper - model.upper_trim,
		lower - model.lower_trim,
	};

	for (int motor = 0; motor < 2; ++motor) {
		next_persistent[motor] =
			model.motor_decay[motor] * persistent[motor]
			+ model.motor_slope_eff[motor] * motor_command_dev[motor];
	}

	if (input.motor_rpm_valid) {
		next_persistent[0] = std::fabs(input.motor_rpm[0]) * model.rpm_to_rad_s - model.upper_trim_rad_s;
		next_persistent[1] = std::fabs(input.motor_rpm[1]) * model.rpm_to_rad_s - model.lower_trim_rad_s;
	}

	// Latent basis filters driven by the final (clamped) servo commands.
	for (int basis = 0; basis < 3; ++basis) {
		const float decay = model.latent_decay[basis];

		for (int axis = 0; axis < 3; ++axis) {
			const int state_index = 2 + 3 * basis + axis;
			float next = decay * persistent[state_index];

			for (int servo = 0; servo < 3; ++servo) {
				next += model.latent_input[(3 * basis + axis) * 3 + servo] * servos[servo];
			}

			next_persistent[state_index] = next;
		}
	}

	for (int axis = 0; axis < 3; ++axis) {
		next_persistent[11 + axis] = integral_next[axis];
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
