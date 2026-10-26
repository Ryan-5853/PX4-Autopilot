/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "NNControl.hpp"

#include "FinalMotorOutputMap.hpp"

#include <cmath>
#include <cstring>

#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

static_assert(static_cast<int32_t>(events::util::hash_32_fnv1a_const("Ground Station")) == -696130346,
	      "update COM_MODE1_HASH in airframe 22001 when the external mode name changes");

static_assert(nn_verification::request::kObservationFirst
	      + ControllerBackend::kVerificationInputSize <= debug_array_s::ARRAY_SIZE,
	      "NN verification request exceeds DEBUG_FLOAT_ARRAY capacity");
static_assert(nn_verification::response::kMacsPerInference
	      < debug_array_s::ARRAY_SIZE,
	      "NN verification response exceeds DEBUG_FLOAT_ARRAY capacity");

NNControl::NNControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

NNControl::~NNControl()
{
	if (_external_mode_registered) {
		unregister_ext_component_s request{};
		request.timestamp = hrt_absolute_time();
		constexpr char mode_name[] = "Ground Station";
		static_assert(sizeof(mode_name) <= sizeof(request.name), "external mode name too long");
		memcpy(request.name, mode_name, sizeof(mode_name));
		request.arming_check_id = _external_arming_check_id;
		request.mode_id = _external_mode_id;
		request.mode_executor_id = -1;
		_unregister_ext_component_pub.publish(request);
	}

	_vehicle_angular_velocity_sub.unregisterCallback();
	perf_free(_cycle_perf);
	perf_free(_inference_perf);
	perf_free(_interval_perf);
	perf_free(_budget_overrun_perf);
	perf_free(_deadline_miss_perf);
	perf_free(_verification_perf);
}

bool NNControl::init()
{
	updateParameters(true);

	if (!_backend.init()) {
		PX4_ERR("%s backend init failed", _backend.name());
		return false;
	}

	// Follow fresh gyro data and cap execution at 500 Hz. For a true 500 Hz
	// loop IMU_GYRO_RATEMAX must be configured to at least 500 Hz (1000 Hz is
	// recommended for deterministic decimation).
	_vehicle_angular_velocity_sub.set_interval_us(kRunInterval);

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("gyro callback registration failed");
		return false;
	}

	// The controller normally runs from fresh gyro callbacks. Keep a slow
	// watchdog schedule as a fallback so stop/start and recovery still work if
	// the sensor voter temporarily has no valid angular-velocity publication.
	ScheduleDelayed(kWatchdogInterval);

	return true;
}

void NNControl::updateSubscriptions()
{
	_actuator_armed_sub.update(&_actuator_armed);
	_battery_status_sub.update(&_battery_status);
	_failsafe_flags_sub.update(&_failsafe_flags);
	_esc_status_sub.update(&_esc_status);
	_input_rc_sub.update(&_input_rc);
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);
	_manual_control_switches_sub.update(&_manual_control_switches);
	_gs_link_control_sub.update(&_gs_link_control);

	_vehicle_acceleration_sub.update(&_vehicle_acceleration);
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_status_sub.update(&_vehicle_status);
}

void NNControl::updateParameters(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();

		actuator_servos_trim_s trims{};
		trims.timestamp = hrt_absolute_time();
		trims.trim[0] = _param_servo_trim_1.get();
		trims.trim[1] = _param_servo_trim_2.get();
		trims.trim[2] = _param_servo_trim_3.get();
		_actuator_servos_trim_pub.publish(trims);
	}
}

void NNControl::updateExternalModeRegistration(hrt_abstime now)
{
	register_ext_component_reply_s reply;
	int max_updates = register_ext_component_reply_s::ORB_QUEUE_LENGTH;

	while (_register_ext_component_reply_sub.update(&reply) && --max_updates >= 0) {
		if (!_external_mode_registered && reply.request_id == _external_mode_request_id) {
			if (reply.success && reply.mode_id >= vehicle_status_s::NAVIGATION_STATE_EXTERNAL1
			    && reply.mode_id <= vehicle_status_s::NAVIGATION_STATE_EXTERNAL8
			    && reply.arming_check_id >= 0) {
				_external_mode_id = reply.mode_id;
				_external_arming_check_id = reply.arming_check_id;
				_external_mode_registered = true;
				publishExternalModeControlMode(now);

				const unsigned external_mode_number =
					reply.mode_id - vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 + 1;
				PX4_INFO("ground station registered as External Mode %u (nav_state %d, arming check %d)",
					 external_mode_number, static_cast<int>(reply.mode_id),
					 static_cast<int>(reply.arming_check_id));

				// Keep RC mode slot 6 mapped to the actual registered external mode.
				// PX4 does not let a component request a specific external mode index.
				const int32_t configured_mode = 100 + static_cast<int32_t>(reply.mode_id)
							       - vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;
				const param_t mode_param = param_find("COM_FLTMODE6");
				int32_t current_mode{-1};

				if (mode_param != PARAM_INVALID && param_get(mode_param, &current_mode) == PX4_OK
				    && current_mode != configured_mode) {
					if (param_set(mode_param, &configured_mode) == PX4_OK) {
						PX4_INFO("RC mode slot 6 mapped to External Mode %u", external_mode_number);
					}
				}

			} else {
				PX4_ERR("ground-station external mode registration rejected");
			}
		}
	}

	arming_check_request_s request;
	max_updates = 4;

	while (_arming_check_request_sub.update(&request) && --max_updates >= 0) {
		if (_external_mode_registered) {
			publishArmingCheckReply(request);
		}
	}

	// Publish once. Retrying the same mode registration after a lost reply can
	// create a second external mode because the first one is already valid in
	// Commander even though the reply was not observed.
	if (!_external_mode_registered && _external_mode_request_id == 0) {
		_external_mode_request_id = static_cast<uint64_t>(hrt_absolute_time()) | 1;

		register_ext_component_request_s mode_request{};
		mode_request.timestamp = now;
		mode_request.request_id = _external_mode_request_id;
		constexpr char mode_name[] = "Ground Station";
		static_assert(sizeof(mode_name) <= sizeof(mode_request.name), "external mode name too long");
		memcpy(mode_request.name, mode_name, sizeof(mode_name));
		mode_request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
		mode_request.register_arming_check = true;
		mode_request.register_mode = true;
		mode_request.register_mode_executor = false;
		mode_request.enable_replace_internal_mode = false;
		mode_request.activate_mode_immediately = false;
		_register_ext_component_request_pub.publish(mode_request);
	}
}

void NNControl::publishExternalModeControlMode(hrt_abstime now)
{
	// Match PX4 Manual mode for a direct controller: sticks are active and the
	// attitude/rate path is in the control-mode contract, but position, altitude
	// and climb-rate control are not. The default external-mode flags enable all
	// of these and make the multicopter land detector wait for a descent setpoint,
	// which prevents a valid RC disarm on a static bench.
	vehicle_control_mode_s control_mode{};
	control_mode.timestamp = now;
	control_mode.source_id = _external_mode_id;
	control_mode.flag_control_manual_enabled = true;
	control_mode.flag_control_attitude_enabled = true;
	control_mode.flag_control_rates_enabled = true;
	control_mode.flag_control_allocation_enabled = true;
	_config_control_setpoints_pub.publish(control_mode);
}

void NNControl::publishArmingCheckReply(const arming_check_request_s &request)
{
	arming_check_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.request_id = request.request_id;
	reply.registration_id = _external_arming_check_id;
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;

	// GS-Link freshness must not participate in PX4's mode availability.
	// Otherwise a short DATA_RC timeout sets mode_req_other and the Commander
	// failsafe falls through RTL/Land/Descend. GroundStationGate owns that
	// timeout and falls back to RC while retaining the selected mode.
	reply.can_arm_and_run = rcManualControlValid(reply.timestamp);
	reply.mode_req_manual_control = true;
	reply.mode_req_attitude = true;
	reply.mode_req_angular_velocity = true;
	_arming_check_reply_pub.publish(reply);
}

bool NNControl::rcManualControlValid(hrt_abstime now)
{
	const bool sample_valid = _manual_control_setpoint.valid
				  && _manual_control_setpoint.timestamp_sample != 0
				  && now >= _manual_control_setpoint.timestamp_sample
				  && now - _manual_control_setpoint.timestamp_sample <= kManualControlTimeout;

	if (!sample_valid) {
		return false;
	}

	const bool hil = _vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON;
	const uint8_t source = _manual_control_setpoint.data_source;

	// HIL keeps the existing WebUI/MAVLink pilot path. Real flight only accepts
	// a physical RC source; MAVLink MANUAL_CONTROL is reserved for the
	// separately gated ground-station submode.
	if (hil) {
		return source == manual_control_setpoint_s::SOURCE_RC
		       || (source >= manual_control_setpoint_s::SOURCE_MAVLINK_0
			   && source <= manual_control_setpoint_s::SOURCE_MAVLINK_5);
	}

	if (source != manual_control_setpoint_s::SOURCE_RC) {
		return false;
	}

	// Reject RC_CHANNELS_OVERRIDE or any other MAVLink-injected "RC" sample.
	// Otherwise a ground station could bypass the channel-8 switch by
	// publishing raw RC channels.
	return _input_rc.timestamp_last_signal != 0
	       && now >= _input_rc.timestamp_last_signal
	       && now - _input_rc.timestamp_last_signal <= kManualControlTimeout
	       && !_input_rc.rc_lost
	       && !_input_rc.rc_failsafe
	       && _input_rc.input_source != input_rc_s::RC_INPUT_SOURCE_UNKNOWN
	       && _input_rc.input_source != input_rc_s::RC_INPUT_SOURCE_MAVLINK;
}

bool NNControl::flightModeSlotValid(hrt_abstime now, uint8_t &mode_slot)
{
	if (!rcManualControlValid(now)
	    || _manual_control_switches.timestamp_sample == 0
	    || now < _manual_control_switches.timestamp_sample
	    || now - _manual_control_switches.timestamp_sample > kManualControlTimeout) {
		return false;
	}

	mode_slot = _manual_control_switches.mode_slot;
	return mode_slot >= manual_control_switches_s::MODE_SLOT_1
	       && mode_slot <= manual_control_switches_s::MODE_SLOT_6;
}

bool NNControl::gsLinkControlValid(hrt_abstime now)
{
	const int32_t timeout_ms = math::constrain<int32_t>(_param_gs_timeout_ms.get(), 50, 1000);
	const hrt_abstime timeout = static_cast<hrt_abstime>(timeout_ms) * 1000;

	if (!_gs_link_control.valid || _gs_link_control.timestamp_sample == 0
	    || now < _gs_link_control.timestamp_sample
	    || now - _gs_link_control.timestamp_sample > timeout) {
		return false;
	}

	return PX4_ISFINITE(_gs_link_control.roll)
	       && PX4_ISFINITE(_gs_link_control.pitch)
	       && PX4_ISFINITE(_gs_link_control.yaw)
	       && PX4_ISFINITE(_gs_link_control.throttle);
}

void NNControl::publishGsLinkStatus(hrt_abstime now)
{
	if (now - _last_gs_status_publish < 20_ms) {
		return;
	}

	gs_link_status_s status{};
	status.timestamp = now;
	status.armed = _actuator_armed.armed;
	status.rc_valid = rcManualControlValid(now);
	status.rc_switch_valid = _flight_mode_slot_valid;
	status.rc_switch_selected = _ground_station_mode_selected;
	status.gs_mode_active = _ground_station_gate.source() == GroundStationGate::Source::GroundStation;
	status.failsafe = _failsafe_flags.manual_control_signal_lost
			  || _failsafe_flags.battery_unhealthy
			  || _failsafe_flags.attitude_invalid
			  || _failsafe_flags.angular_velocity_invalid
			  || _failsafe_flags.fd_critical_failure;
	status.command_busy = false;
	status.takeoff_active = false;
	status.landing_active = false;
	status.rc_switch = static_cast<float>(_flight_mode_slot);
	_gs_link_status_pub.publish(status);
	_last_gs_status_publish = now;
}

void NNControl::updateManualSource(hrt_abstime now)
{
	const bool rc_valid = rcManualControlValid(now);
	const bool manual_fresh = _manual_control_setpoint.valid
				  && _manual_control_setpoint.timestamp_sample != 0
				  && now >= _manual_control_setpoint.timestamp_sample
				  && now - _manual_control_setpoint.timestamp_sample <= kManualControlTimeout;
	_manual_control_wrong_source = manual_fresh && !rc_valid;

	_flight_mode_slot_valid = flightModeSlotValid(now, _flight_mode_slot);
	_gs_link_input_valid = gsLinkControlValid(now);

	GroundStationGate::Config config{};
	config.enabled = _param_gs_enable.get() != 0;
	const int32_t configured_mode_slot = _param_gs_mode_slot.get();
	config.mode_slot = configured_mode_slot >= manual_control_switches_s::MODE_SLOT_1
			   && configured_mode_slot <= manual_control_switches_s::MODE_SLOT_6
			   ? static_cast<uint8_t>(configured_mode_slot) : 0;
	config.entry_delay_us = 100_ms;

	const bool requested_ground_station = _flight_mode_slot_valid
					     && _flight_mode_slot == config.mode_slot;
	const bool external_mode_active = _external_mode_registered
					 && _vehicle_status.nav_state == _external_mode_id;
	_ground_station_mode_selected = requested_ground_station && external_mode_active;

	GroundStationGate::Input input{};
	input.now = now;
	input.armed = _actuator_armed.armed;
	input.rc_valid = rc_valid;
	input.mode_slot_valid = _flight_mode_slot_valid;
	input.mode_slot = _flight_mode_slot;
	input.external_mode_active = external_mode_active;
	input.ground_station_valid = _gs_link_input_valid;

	const GroundStationGate::Source previous_source = _ground_station_gate.source();
	const GroundStationGate::Source source = _ground_station_gate.update(config, input);

	switch (source) {
	case GroundStationGate::Source::GroundStation:
		_effective_manual_control = {};
		_effective_manual_control.timestamp = now;
		_effective_manual_control.timestamp_sample = _gs_link_control.timestamp_sample;
		_effective_manual_control.valid = true;
		_effective_manual_control.roll = _gs_link_control.roll;
		_effective_manual_control.pitch = _gs_link_control.pitch;
		_effective_manual_control.yaw = _gs_link_control.yaw;
		_effective_manual_control.throttle = _gs_link_control.throttle;
		_manual_command_valid = _gs_link_input_valid;
		break;

	case GroundStationGate::Source::Rc:
		_effective_manual_control = _manual_control_setpoint;
		_manual_command_valid = rc_valid;
		break;

	default:
		_effective_manual_control = {};
		_manual_command_valid = false;
		break;
	}

	if (source != previous_source) {
		if (source == GroundStationGate::Source::GroundStation) {
			PX4_INFO("manual source: ground station (GS-Link), RC mode slot %u",
				 static_cast<unsigned>(_flight_mode_slot));

		} else if (previous_source == GroundStationGate::Source::GroundStation) {
			PX4_INFO("manual source: RC (ground-station control interrupted)");
		}
	}
}

void NNControl::handleFlightPhaseTransitions()
{
	const bool armed = _actuator_armed.armed;

	// 上锁后清空整个控制器状态，等待下一次解锁-上油门的起飞流程。
	if (_last_armed && !armed) {
		_backend.reset();
		_last_sample_timestamp = 0;
		_takeoff_done = false;
		PX4_INFO("disarm: controller state cleared");
	}

	if (armed && !_last_armed) {
		// 解锁本身不重置状态，只开始新一轮“等待第一次油门输出”。
		_takeoff_done = false;
	}

	// 起飞事件 = 解锁且油门有输出（与 LQI idle 门控的判据一致）。
	// 此时整体重置控制器：LQI 积分/执行器观测器归零，目标 yaw 在同一
	// 控制帧从当前航向重新初始化；之后只要不打航向杆，目标航向就保持
	// 起飞瞬间的航向。
	const float throttle = _manual_command_valid
			       ? math::constrain(_effective_manual_control.throttle, -1.0f, 1.0f) : 0.f;
	const bool throttle_active = (throttle + 1.0f) * 0.5f > 0.0f;
	const bool output_gate = armed && throttle_active;

	if (output_gate && !_takeoff_done) {
		_backend.reset();
		_last_sample_timestamp = 0;
		_takeoff_done = true;
		PX4_INFO("takeoff: controller state reset, yaw reference = current heading");
	}

	_last_armed = armed;
}

bool NNControl::buildInput(ControllerInput &input, hrt_abstime now)
{
	if (_vehicle_attitude.timestamp_sample == 0
	    || now - _vehicle_attitude.timestamp_sample > kEstimatorTimeout) {
		_inhibit_reason = InhibitReason::StaleAttitude;
		return false;
	}

	if (_vehicle_angular_velocity.timestamp_sample == 0
	    || now - _vehicle_angular_velocity.timestamp_sample > kEstimatorTimeout) {
		_inhibit_reason = InhibitReason::StaleAngularVelocity;
		return false;
	}

	if (_vehicle_acceleration.timestamp_sample == 0
	    || now - _vehicle_acceleration.timestamp_sample > kEstimatorTimeout) {
		_inhibit_reason = InhibitReason::StaleAcceleration;
		return false;
	}

	input = {};
	input.timestamp = now;
	input.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	memcpy(input.attitude_q, _vehicle_attitude.q, sizeof(input.attitude_q));
	memcpy(input.angular_velocity, _vehicle_angular_velocity.xyz, sizeof(input.angular_velocity));
	memcpy(input.angular_acceleration, _vehicle_angular_velocity.xyz_derivative,
	       sizeof(input.angular_acceleration));
	memcpy(input.acceleration, _vehicle_acceleration.xyz, sizeof(input.acceleration));

	input.local_position[0] = _vehicle_local_position.x;
	input.local_position[1] = _vehicle_local_position.y;
	input.local_position[2] = _vehicle_local_position.z;
	input.local_velocity[0] = _vehicle_local_position.vx;
	input.local_velocity[1] = _vehicle_local_position.vy;
	input.local_velocity[2] = _vehicle_local_position.vz;
	input.heading = _vehicle_local_position.heading;
	input.local_position_valid = _vehicle_local_position.xy_valid && _vehicle_local_position.z_valid;
	input.local_velocity_valid = _vehicle_local_position.v_xy_valid && _vehicle_local_position.v_z_valid;

	// Optional estimator fields are represented by validity flags, never NaN,
	// so a model cannot accidentally propagate an unavailable feature.
	if (!input.local_position_valid) {
		memset(input.local_position, 0, sizeof(input.local_position));
	}

	if (!input.local_velocity_valid) {
		memset(input.local_velocity, 0, sizeof(input.local_velocity));
	}

	if (!PX4_ISFINITE(input.heading)) {
		input.heading = 0.f;
	}

	if (!_manual_command_valid) {
		_inhibit_reason = _manual_control_wrong_source
				  ? InhibitReason::ManualControlNotRc : InhibitReason::StaleManualControl;
		return false;
	}

	input.rc_roll = _effective_manual_control.roll;
	input.rc_pitch = _effective_manual_control.pitch;
	input.rc_yaw = _effective_manual_control.yaw;
	input.rc_throttle = _effective_manual_control.throttle;
	input.rc_aux[0] = _manual_control_setpoint.aux1;
	input.rc_aux[1] = _manual_control_setpoint.aux2;
	input.rc_aux[2] = _manual_control_setpoint.aux3;
	input.rc_aux[3] = _manual_control_setpoint.aux4;
	input.rc_aux[4] = _manual_control_setpoint.aux5;
	input.rc_aux[5] = _manual_control_setpoint.aux6;

	for (float &value : input.rc_aux) {
		if (!PX4_ISFINITE(value)) {
			value = 0.f;
		}
	}

	// ESC feedback is optional. Match by actuator function instead of array
	// position because telemetry order is driver-dependent.
	bool motor_found[2] {false, false};

	if (_esc_status.timestamp != 0 && now - _esc_status.timestamp <= kEstimatorTimeout) {
		for (int esc_index = 0; esc_index < _esc_status.esc_count
		     && esc_index < esc_status_s::CONNECTED_ESC_MAX; ++esc_index) {
			if ((_esc_status.esc_online_flags & (1u << esc_index)) == 0) {
				continue;
			}

			const int motor_index = static_cast<int>(_esc_status.esc[esc_index].actuator_function)
					- static_cast<int>(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1);

			if (motor_index >= 0 && motor_index < 2) {
				input.motor_rpm[motor_index] = static_cast<float>(_esc_status.esc[esc_index].esc_rpm);
				motor_found[motor_index] = true;
			}
		}
	}

	input.motor_rpm_valid = motor_found[0] && motor_found[1];

	input.battery_voltage_v = PX4_ISFINITE(_battery_status.voltage_v) ? _battery_status.voltage_v : 0.f;
	input.battery_current_a = PX4_ISFINITE(_battery_status.current_a) ? _battery_status.current_a : -1.f;
	input.battery_remaining = PX4_ISFINITE(_battery_status.remaining) ? _battery_status.remaining : 0.f;

	for (float value : input.attitude_q) {
		if (!PX4_ISFINITE(value)) {
			_inhibit_reason = InhibitReason::InvalidInput;
			return false;
		}
	}

	for (float value : input.angular_velocity) {
		if (!PX4_ISFINITE(value)) {
			_inhibit_reason = InhibitReason::InvalidInput;
			return false;
		}
	}

	for (float value : input.angular_acceleration) {
		if (!PX4_ISFINITE(value)) {
			_inhibit_reason = InhibitReason::InvalidInput;
			return false;
		}
	}

	for (float value : input.acceleration) {
		if (!PX4_ISFINITE(value)) {
			_inhibit_reason = InhibitReason::InvalidInput;
			return false;
		}
	}

	if (!PX4_ISFINITE(input.rc_roll) || !PX4_ISFINITE(input.rc_pitch)
	    || !PX4_ISFINITE(input.rc_yaw) || !PX4_ISFINITE(input.rc_throttle)) {
		_inhibit_reason = InhibitReason::InvalidInput;
		return false;
	}

	return true;
}

bool NNControl::actuatorAuthorityAllowed()
{
	if (!_actuator_armed.armed) {
		_inhibit_reason = InhibitReason::Disarmed;
		return false;
	}

	// Commander deliberately asserts automatic lockdown in HIL so no physical
	// output driver can move hardware. pwm_out_sim is the only HIL consumer and
	// explicitly ignores this automatic lockdown. Keep every manual/failsafe
	// lock, but allow the simulated actuator publication while HIL is active.
	const bool automatic_lockdown = _actuator_armed.lockdown
					&& _vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON;

	if (automatic_lockdown || _actuator_armed.manual_lockdown
	    || _actuator_armed.force_failsafe || _actuator_armed.in_esc_calibration_mode) {
		_inhibit_reason = InhibitReason::ActuatorLockdown;
		return false;
	}

	// HIL direct-control inputs are validated above from the fresh uORB
	// samples themselves. Commander estimator flags can remain invalid while
	// the injected attitude/gyro stream is valid (there is no EKF contract for
	// this controller), so do not reject an otherwise valid HIL command on
	// those redundant flags. Keep pilot, power and hard failure gates active.
	const bool estimator_failure = _failsafe_flags.attitude_invalid
					|| _failsafe_flags.angular_velocity_invalid;
	const bool estimator_failure_blocks = _vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON
					&& estimator_failure;

	const bool battery_failure_blocks = _vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON
					&& (_failsafe_flags.battery_unhealthy
					|| _failsafe_flags.battery_warning >= battery_status_s::WARNING_CRITICAL);

	if (estimator_failure_blocks
	    || _failsafe_flags.manual_control_signal_lost
	    || battery_failure_blocks
	    || _failsafe_flags.fd_critical_failure
	    || _failsafe_flags.fd_esc_arming_failure
	    || _failsafe_flags.fd_motor_failure) {
		_inhibit_reason = InhibitReason::Failsafe;
		return false;
	}

	return true;
}

bool NNControl::validateOutput(const ActuatorSetpoint &output)
{
	for (float value : output.motors) {
		if (!PX4_ISFINITE(value) || value < 0.f || value > 1.f) {
			_inhibit_reason = InhibitReason::InvalidOutput;
			return false;
		}
	}

	for (float value : output.servos) {
		if (!PX4_ISFINITE(value) || value < -1.f || value > 1.f) {
			_inhibit_reason = InhibitReason::InvalidOutput;
			return false;
		}
	}

	return true;
}

void NNControl::publishOutput(const ActuatorSetpoint *output, hrt_abstime now)
{
	actuator_motors_s motors{};
	motors.timestamp = now;
	motors.timestamp_sample = now;

	for (float &control : motors.control) {
		control = NAN;
	}

	actuator_servos_s servos{};
	servos.timestamp = now;
	servos.timestamp_sample = now;

	for (float &control : servos.control) {
		control = NAN;
	}

	if (output != nullptr) {
		for (int i = 0; i < ActuatorSetpoint::kMotorCount; ++i) {
			motors.control[i] = output->motors[i];
		}

		const float trims[ActuatorSetpoint::kServoCount] {
			_param_servo_trim_1.get(),
			_param_servo_trim_2.get(),
			_param_servo_trim_3.get()
		};

		for (int i = 0; i < ActuatorSetpoint::kServoCount; ++i) {
			servos.control[i] = math::constrain(output->servos[i] + trims[i], -1.f, 1.f);
		}
	}

	_actuator_motors_pub.publish(motors);
	_actuator_servos_pub.publish(servos);
}

void NNControl::publishVerificationResponse(uint32_t sequence,
		nn_verification::Status status,
		hrt_abstime inference_time,
		const float *output)
{
	debug_array_s response{};
	response.timestamp = hrt_absolute_time();
	response.id = nn_verification::kResponseId;
	strncpy(response.name, nn_verification::kResponseName, sizeof(response.name));
	response.data[nn_verification::response::kVersion] = nn_verification::kProtocolVersion;
	response.data[nn_verification::response::kStatus] = static_cast<uint8_t>(status);
	response.data[nn_verification::response::kSequence] = static_cast<float>(sequence);
	response.data[nn_verification::response::kInferenceTimeUs] = static_cast<float>(inference_time);

	if (output != nullptr) {
		for (int i = 0; i < ControllerBackend::kVerificationOutputSize; ++i) {
			response.data[nn_verification::response::kOutputFirst + i] = output[i];
		}
	}

	const uint32_t checksum = _backend.verificationChecksum();
	response.data[nn_verification::response::kChecksumLow] = nn_verification::checksumLow(checksum);
	response.data[nn_verification::response::kChecksumHigh] = nn_verification::checksumHigh(checksum);
	response.data[nn_verification::response::kWeightBytes] =
		static_cast<float>(_backend.verificationWeightBytes());
	response.data[nn_verification::response::kMacsPerInference] =
		static_cast<float>(_backend.verificationMacsPerInference());
	_verification_response_pub.publish(response);
}

void NNControl::handleVerificationRequest()
{
	debug_array_s request{};

	if (!_verification_request_sub.update(&request)
	    || request.id != nn_verification::kRequestId
	    || strncmp(request.name, nn_verification::kRequestName,
		       sizeof(nn_verification::kRequestName) - 1) != 0) {
		return;
	}

	++_verification_requests;

	if (!_backend.verificationSupported()) {
		++_verification_failures;
		publishVerificationResponse(0, nn_verification::Status::InferenceFailed, 0, nullptr);
		return;
	}

	for (int index = nn_verification::request::kVersion;
	     index <= nn_verification::request::kChecksumHigh; ++index) {
		if (!PX4_ISFINITE(request.data[index])) {
			++_verification_failures;
			publishVerificationResponse(0, nn_verification::Status::InvalidObservation, 0, nullptr);
			return;
		}
	}

	if (request.data[nn_verification::request::kVersion] < 0.f
	    || request.data[nn_verification::request::kVersion] > 255.f
	    || request.data[nn_verification::request::kCommand] < 0.f
	    || request.data[nn_verification::request::kCommand] > 255.f
	    || request.data[nn_verification::request::kSequence] < 0.f
	    || request.data[nn_verification::request::kSequence] > 16777215.f
	    || request.data[nn_verification::request::kChecksumLow] < 0.f
	    || request.data[nn_verification::request::kChecksumLow] > 65535.f
	    || request.data[nn_verification::request::kChecksumHigh] < 0.f
	    || request.data[nn_verification::request::kChecksumHigh] > 65535.f) {
		++_verification_failures;
		publishVerificationResponse(0, nn_verification::Status::InvalidObservation, 0, nullptr);
		return;
	}

	const uint32_t sequence = static_cast<uint32_t>(
					  request.data[nn_verification::request::kSequence]);
	const uint8_t version = static_cast<uint8_t>(
					request.data[nn_verification::request::kVersion]);
	const auto command = static_cast<nn_verification::Command>(
				     static_cast<uint8_t>(request.data[nn_verification::request::kCommand]));
	const uint32_t expected_checksum = nn_verification::checksumFromFloatWords(
			request.data[nn_verification::request::kChecksumLow],
			request.data[nn_verification::request::kChecksumHigh]);

	if (version != nn_verification::kProtocolVersion) {
		++_verification_failures;
		publishVerificationResponse(sequence, nn_verification::Status::BadVersion, 0, nullptr);
		return;
	}

	if (expected_checksum != _backend.verificationChecksum()) {
		++_verification_failures;
		publishVerificationResponse(sequence, nn_verification::Status::BadChecksum, 0, nullptr);
		return;
	}

	switch (command) {
	case nn_verification::Command::Info:
		publishVerificationResponse(sequence, nn_verification::Status::Ok, 0, nullptr);
		return;

	case nn_verification::Command::Reset:
		publishVerificationResponse(sequence,
			_backend.verificationReset() ? nn_verification::Status::Ok
			: nn_verification::Status::InferenceFailed,
			0, nullptr);
		return;

	case nn_verification::Command::Step: {
			float observations[ControllerBackend::kVerificationInputSize] {};

			for (int i = 0; i < ControllerBackend::kVerificationInputSize; ++i) {
				observations[i] =
					request.data[nn_verification::request::kObservationFirst + i];

				if (!PX4_ISFINITE(observations[i])) {
					++_verification_failures;
					publishVerificationResponse(sequence,
								    nn_verification::Status::InvalidObservation,
								    0, nullptr);
					return;
				}
			}

			float output[ControllerBackend::kVerificationOutputSize] {};
			const hrt_abstime inference_start = hrt_absolute_time();
			perf_begin(_verification_perf);
			const bool success = _backend.verificationStep(observations, output);
			perf_end(_verification_perf);
			_last_verification_time = hrt_elapsed_time(&inference_start);
			_max_verification_time = math::max(_max_verification_time, _last_verification_time);

			bool output_valid = success;

			for (float value : output) {
				output_valid = output_valid && PX4_ISFINITE(value) && value >= -1.f && value <= 1.f;
			}

			if (!output_valid) {
				++_verification_failures;
				publishVerificationResponse(sequence,
							    nn_verification::Status::InferenceFailed,
							    _last_verification_time, nullptr);
				return;
			}

			publishVerificationResponse(sequence, nn_verification::Status::Ok,
						    _last_verification_time, output);
			return;
		}
	}

	++_verification_failures;
	publishVerificationResponse(sequence, nn_verification::Status::UnknownCommand, 0, nullptr);
}

void NNControl::Run()
{
	// Re-arm the fallback watchdog on every invocation. A gyro callback can
	// still wake this work item at the normal 500 Hz rate.
	ScheduleDelayed(kWatchdogInterval);

	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		ScheduleClear();
		publishOutput(nullptr, hrt_absolute_time());
		exit_and_cleanup();
		return;
	}

	const hrt_abstime cycle_start = hrt_absolute_time();
	perf_begin(_cycle_perf);
	perf_count(_interval_perf);
	updateSubscriptions();
	updateParameters();
	updateExternalModeRegistration(cycle_start);
	updateManualSource(cycle_start);
	handleFlightPhaseTransitions();
	publishGsLinkStatus(cycle_start);
	handleVerificationRequest();

	ControllerInput input{};
	ActuatorSetpoint output{};
	bool inference_valid = buildInput(input, cycle_start);

	if (!inference_valid && _last_input_valid) {
		_backend.reset();
	}

	_last_input_valid = inference_valid;

	if (inference_valid && input.timestamp_sample == _last_sample_timestamp) {
		inference_valid = false;
		_inhibit_reason = InhibitReason::StaleAngularVelocity;
	}

	if (inference_valid) {
		_last_sample_timestamp = input.timestamp_sample;
		++_inference_count;
		const hrt_abstime inference_start = hrt_absolute_time();
		perf_begin(_inference_perf);

		if (!_backend.run(input, output)) {
			_inhibit_reason = InhibitReason::BackendUnavailable;
			inference_valid = false;
		}

		perf_end(_inference_perf);
		_last_inference_time = hrt_elapsed_time(&inference_start);
		_max_inference_time = math::max(_max_inference_time, _last_inference_time);

		if (_last_inference_time > kInferenceBudget) {
			++_inference_overruns;
		}
	}

	if (inference_valid) {
		inference_valid = validateOutput(output);

		// Validate the controller-domain output before calibration so an
		// out-of-range command cannot be hidden by the final scaling.
		if (inference_valid) {
			final_motor_output_map::apply(output);
		}
	}

	bool authority = false;

	if (inference_valid) {
		_last_output = output;
		authority = actuatorAuthorityAllowed();
	}

	const bool actuator_output_allowed = inference_valid && authority
					&& _backend.actuatorOutputAllowed();
	const bool hold_hil_timestamp = _vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON
					&& _actuator_armed.armed && !actuator_output_allowed;

	if (actuator_output_allowed) {
		_inhibit_reason = InhibitReason::None;
		publishOutput(&output, cycle_start);
		_output_enabled = true;

	} else {
		if (inference_valid && !_backend.actuatorOutputAllowed()) {
			_inhibit_reason = InhibitReason::BackendOutputDisabled;
		}

		++_rejected_count;
		// In HIL, do not publish a new timestamp for an armed but invalid
		// controller input. The host transport must observe the stale boundary
		// and stop advancing the plant instead of treating zero/NaN output as a
		// valid control frame. Disarmed operation still publishes a fresh null
		// frame so the startup handshake can observe the unarmed state.
		if (!hold_hil_timestamp) {
			publishOutput(nullptr, cycle_start);
		}
		_output_enabled = false;
	}

	_last_cycle_time = hrt_elapsed_time(&cycle_start);
	_max_cycle_time = math::max(_max_cycle_time, _last_cycle_time);

	if (_last_cycle_time > kCycleBudget) {
		++_cycle_overruns;
		perf_count(_budget_overrun_perf);
	}

	if (_last_cycle_time > kHardDeadline) {
		++_hard_deadline_misses;
		perf_count(_deadline_miss_perf);
	}

	perf_end(_cycle_perf);
}

const char *NNControl::reasonString(InhibitReason reason)
{
	switch (reason) {
	case InhibitReason::None: return "none";
	case InhibitReason::Disarmed: return "disarmed";
	case InhibitReason::ActuatorLockdown: return "actuator lockdown";
	case InhibitReason::Failsafe: return "failsafe";
	case InhibitReason::StaleAttitude: return "attitude missing/stale";
	case InhibitReason::StaleAngularVelocity: return "angular velocity missing/stale";
	case InhibitReason::StaleAcceleration: return "acceleration missing/stale";
	case InhibitReason::StaleManualControl: return "manual control missing/stale";
	case InhibitReason::ManualControlNotRc: return "manual control is not RC";
	case InhibitReason::InvalidInput: return "non-finite input";
	case InhibitReason::BackendUnavailable: return "backend unavailable/rejected";
	case InhibitReason::InvalidOutput: return "invalid controller output";
	case InhibitReason::BackendOutputDisabled: return "backend output disabled";
	}

	return "unknown";
}

int NNControl::task_spawn(int argc, char *argv[])
{
	NNControl *instance = new NNControl();

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	PX4_ERR("alloc/init failed");
	return PX4_ERROR;
}

int NNControl::print_status()
{
	const float interval_s = perf_mean(_interval_perf);
	const float measured_rate_hz = interval_s > 0.f ? 1.f / interval_s : 0.f;
	const auto source = _ground_station_gate.source();
	const char *source_name = "none";

	if (source == GroundStationGate::Source::Rc) {
		source_name = "RC";

	} else if (source == GroundStationGate::Source::GroundStation) {
		source_name = "ground station";
	}

	PX4_INFO("backend: %s, %s", _backend.name(), _backend.mode());
	PX4_INFO("manual source: %s, RC data source: %u, GS enabled: %d",
		 source_name, (unsigned)_manual_control_setpoint.data_source,
		 (int)_param_gs_enable.get());
	PX4_INFO("RC flight mode: slot %u, target slot %ld, valid %d, GS slot selected %d",
		 (unsigned)_flight_mode_slot, (long)_param_gs_mode_slot.get(),
		 _flight_mode_slot_valid ? 1 : 0, _ground_station_mode_selected ? 1 : 0);
	PX4_INFO("external mode: %s, id %d, arming check %d",
		 _external_mode_registered ? "registered" : "not registered",
		 static_cast<int>(_external_mode_id), static_cast<int>(_external_arming_check_id));
	PX4_INFO("ground station: GS-Link %s, age %lld ms, quality %u, timeout %ld ms",
		 _gs_link_input_valid ? "fresh" : "missing/stale",
		 (long long)(_gs_link_control.timestamp_sample == 0 ? -1
			     : (long long)(hrt_elapsed_time(&_gs_link_control.timestamp_sample) / 1000)),
		 (unsigned)_gs_link_control.link_quality,
		 (long)_param_gs_timeout_ms.get());
	PX4_INFO("target 500 Hz, measured %.1f Hz", (double)measured_rate_hz);
	PX4_INFO("verification weights: %lu bytes, checksum: 0x%08lx",
		 (unsigned long)_backend.verificationWeightBytes(),
		 (unsigned long)_backend.verificationChecksum());
	PX4_INFO("actuator output: %s", _output_enabled ? "ENABLED" : "inhibited");
	PX4_INFO("inhibit reason: %s", reasonString(_inhibit_reason));
	PX4_INFO("inferences: %lu, rejected cycles: %lu",
		 (unsigned long)_inference_count, (unsigned long)_rejected_count);
	PX4_INFO("inference last/max: %llu/%llu us, >1 ms: %lu",
		 (unsigned long long)_last_inference_time, (unsigned long long)_max_inference_time,
		 (unsigned long)_inference_overruns);
	PX4_INFO("cycle last/max: %llu/%llu us, >1.5 ms: %lu, >2 ms: %lu",
		 (unsigned long long)_last_cycle_time, (unsigned long long)_max_cycle_time,
		 (unsigned long)_cycle_overruns, (unsigned long)_hard_deadline_misses);
	PX4_INFO("last motors: [%.4f %.4f], servos: [%.4f %.4f %.4f]",
		 (double)_last_output.motors[0], (double)_last_output.motors[1],
		 (double)_last_output.servos[0], (double)_last_output.servos[1],
		 (double)_last_output.servos[2]);
	PX4_INFO("servo trims: [%.3f %.3f %.3f]",
		 (double)_param_servo_trim_1.get(), (double)_param_servo_trim_2.get(),
		 (double)_param_servo_trim_3.get());
	PX4_INFO("MAVLink verification: requests %lu, failures %lu, last/max %llu/%llu us",
		 (unsigned long)_verification_requests, (unsigned long)_verification_failures,
		 (unsigned long long)_last_verification_time,
		 (unsigned long long)_max_verification_time);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_inference_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_budget_overrun_perf);
	perf_print_counter(_deadline_miss_perf);
	perf_print_counter(_verification_perf);
	return 0;
}

void NNControl::resetDebug()
{
	_backend.reset();
	_inference_count = 0;
	_rejected_count = 0;
	_inference_overruns = 0;
	_cycle_overruns = 0;
	_hard_deadline_misses = 0;
	_verification_requests = 0;
	_verification_failures = 0;
	_last_verification_time = 0;
	_max_verification_time = 0;
	_last_inference_time = 0;
	_max_inference_time = 0;
	_last_cycle_time = 0;
	_max_cycle_time = 0;
	_last_sample_timestamp = 0;
	_last_output = {};
	perf_reset(_cycle_perf);
	perf_reset(_inference_perf);
	perf_reset(_interval_perf);
	perf_reset(_budget_overrun_perf);
	perf_reset(_deadline_miss_perf);
	perf_reset(_verification_perf);
	PX4_INFO("%s backend state and load counters reset", _backend.name());
}

int NNControl::custom_command(int argc, char *argv[])
{
	if (argc > 0 && !strcmp(argv[0], "reset")) {
		if (get_instance() != nullptr) {
			get_instance()->resetDebug();
			return PX4_OK;
		}

		return PX4_ERROR;
	}

	return print_usage("unknown command");
}

int NNControl::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Compile-time-selectable direct flight-controller interface.

The framework gathers estimator, IMU, pilot and battery data from uORB and runs
the selected neural, LQR or hybrid backend from fresh gyro callbacks. Every
backend produces the same normalized two-motor/three-servo actuator contract.

Set IMU_GYRO_RATEMAX to 1000 for a 500 Hz callback-limited benchmark. Use
`nn_control status` to inspect measured rate, inference/cycle latency and
deadline misses. Scaffold or dry-run backends publish NaN actuator controls.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nn_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset backend state and load counters");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int nn_control_main(int argc, char *argv[])
{
	return NNControl::main(argc, argv);
}
