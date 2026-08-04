/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

/**
 * Stable input contract between PX4/uORB and the selected controller backend.
 *
 * Quaternion order is [w, x, y, z]. Angular quantities and acceleration use
 * body FRD, local position and velocity use NED, and manual axes use [-1, 1].
 */
struct ControllerInput {
	uint64_t timestamp;
	uint64_t timestamp_sample;

	float attitude_q[4];
	float angular_velocity[3];
	float angular_acceleration[3];
	float acceleration[3];

	float local_position[3];
	float local_velocity[3];
	float heading;
	bool local_position_valid;
	bool local_velocity_valid;

	float rc_roll;
	float rc_pitch;
	float rc_yaw;
	float rc_throttle;
	float rc_aux[6];

	float battery_voltage_v;
	float battery_current_a;
	float battery_remaining;
};

/**
 * Stable normalized actuator contract for every controller backend.
 *
 * Non-reversible motors use [0, 1]. Servos use [-1, 1]. The common framework
 * applies normalized servo trim; the PX4 output driver applies physical
 * direction, minimum, maximum, disarmed and failsafe values.
 */
struct ActuatorSetpoint {
	static constexpr int kMotorCount{2};
	static constexpr int kServoCount{3};

	float motors[kMotorCount];
	float servos[kServoCount];
};
