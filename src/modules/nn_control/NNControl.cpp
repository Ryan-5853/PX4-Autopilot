/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "NNControl.hpp"

#include <cmath>
#include <cstring>

#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

static_assert(nn_verification::request::kObservationFirst
	      + NeuralNetworkInterface::kInputSize <= debug_array_s::ARRAY_SIZE,
	      "NN verification request exceeds DEBUG_FLOAT_ARRAY capacity");
static_assert(nn_verification::response::kMacsPerInference
	      < debug_array_s::ARRAY_SIZE,
	      "NN verification response exceeds DEBUG_FLOAT_ARRAY capacity");

NNControl::NNControl() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

NNControl::~NNControl()
{
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
	if (!_network.init()) {
		PX4_ERR("network init failed");
		return false;
	}

	if (!_verification_network.init()
	    || _verification_network.weightChecksum() != _network.weightChecksum()) {
		PX4_ERR("verification network init/checksum failed");
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

	return true;
}

void NNControl::updateSubscriptions()
{
	_actuator_armed_sub.update(&_actuator_armed);
	_battery_status_sub.update(&_battery_status);
	_failsafe_flags_sub.update(&_failsafe_flags);
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);
	_vehicle_acceleration_sub.update(&_vehicle_acceleration);
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
}

bool NNControl::buildInput(NeuralControlInput &input, hrt_abstime now)
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

	const bool manual_valid = _manual_control_setpoint.valid
				  && _manual_control_setpoint.timestamp_sample != 0
				  && now - _manual_control_setpoint.timestamp_sample <= kManualControlTimeout;

	if (manual_valid) {
		input.rc_roll = _manual_control_setpoint.roll;
		input.rc_pitch = _manual_control_setpoint.pitch;
		input.rc_yaw = _manual_control_setpoint.yaw;
		input.rc_throttle = _manual_control_setpoint.throttle;
		input.rc_aux[0] = _manual_control_setpoint.aux1;
		input.rc_aux[1] = _manual_control_setpoint.aux2;
		input.rc_aux[2] = _manual_control_setpoint.aux3;
		input.rc_aux[3] = _manual_control_setpoint.aux4;
		input.rc_aux[4] = _manual_control_setpoint.aux5;
		input.rc_aux[5] = _manual_control_setpoint.aux6;
	}

	for (float &value : input.rc_aux) {
		if (!PX4_ISFINITE(value)) {
			value = 0.f;
		}
	}

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

	if (_actuator_armed.lockdown || _actuator_armed.manual_lockdown
	    || _actuator_armed.force_failsafe || _actuator_armed.in_esc_calibration_mode) {
		_inhibit_reason = InhibitReason::ActuatorLockdown;
		return false;
	}

	if (_failsafe_flags.attitude_invalid
	    || _failsafe_flags.angular_velocity_invalid
	    || _failsafe_flags.manual_control_signal_lost
	    || _failsafe_flags.battery_unhealthy
	    || _failsafe_flags.battery_warning >= battery_status_s::WARNING_CRITICAL
	    || _failsafe_flags.fd_critical_failure
	    || _failsafe_flags.fd_esc_arming_failure
	    || _failsafe_flags.fd_motor_failure) {
		_inhibit_reason = InhibitReason::Failsafe;
		return false;
	}

	return true;
}

bool NNControl::validateOutput(const NeuralControlOutput &output)
{
	for (float value : output.controls) {
		if (!PX4_ISFINITE(value) || value < -1.f || value > 1.f) {
			_inhibit_reason = InhibitReason::InvalidOutput;
			return false;
		}
	}

	return true;
}

void NNControl::publishOutput(const NeuralControlOutput *output, hrt_abstime now)
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
		// Placeholder four-output mapping: signed network outputs are converted
		// to non-reversible motor thrust for channels 0/1; channels 2/3 remain
		// signed servo commands. Dry-run prevents this path for random weights.
		motors.control[0] = math::constrain((output->controls[0] + 1.f) * 0.5f, 0.f, 1.f);
		motors.control[1] = math::constrain((output->controls[1] + 1.f) * 0.5f, 0.f, 1.f);
		servos.control[0] = output->controls[2];
		servos.control[1] = output->controls[3];
	}

	_actuator_motors_pub.publish(motors);
	_actuator_servos_pub.publish(servos);
}

void NNControl::publishVerificationResponse(uint32_t sequence,
		nn_verification::Status status,
		hrt_abstime inference_time,
		const NeuralControlOutput *output)
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
		for (int i = 0; i < NeuralNetworkInterface::kOutputSize; ++i) {
			response.data[nn_verification::response::kOutputFirst + i] = output->controls[i];
		}
	}

	const uint32_t checksum = _verification_network.weightChecksum();
	response.data[nn_verification::response::kChecksumLow] = nn_verification::checksumLow(checksum);
	response.data[nn_verification::response::kChecksumHigh] = nn_verification::checksumHigh(checksum);
	response.data[nn_verification::response::kWeightBytes] =
		static_cast<float>(NeuralNetworkInterface::weightBytes());
	response.data[nn_verification::response::kMacsPerInference] =
		static_cast<float>(NeuralNetworkInterface::kMacsPerInference);
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

	if (expected_checksum != _verification_network.weightChecksum()) {
		++_verification_failures;
		publishVerificationResponse(sequence, nn_verification::Status::BadChecksum, 0, nullptr);
		return;
	}

	switch (command) {
	case nn_verification::Command::Info:
		publishVerificationResponse(sequence, nn_verification::Status::Ok, 0, nullptr);
		return;

	case nn_verification::Command::Reset:
		_verification_network.reset();
		publishVerificationResponse(sequence, nn_verification::Status::Ok, 0, nullptr);
		return;

	case nn_verification::Command::Step: {
			float observations[NeuralNetworkInterface::kInputSize] {};

			for (int i = 0; i < NeuralNetworkInterface::kInputSize; ++i) {
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

			NeuralControlOutput output{};
			const hrt_abstime inference_start = hrt_absolute_time();
			perf_begin(_verification_perf);
			const bool success = _verification_network.runFeatures(observations, output);
			perf_end(_verification_perf);
			_last_verification_time = hrt_elapsed_time(&inference_start);
			_max_verification_time = math::max(_max_verification_time, _last_verification_time);

			if (!success || !validateOutput(output)) {
				++_verification_failures;
				publishVerificationResponse(sequence,
							    nn_verification::Status::InferenceFailed,
							    _last_verification_time, nullptr);
				return;
			}

			publishVerificationResponse(sequence, nn_verification::Status::Ok,
						    _last_verification_time, &output);
			return;
		}
	}

	++_verification_failures;
	publishVerificationResponse(sequence, nn_verification::Status::UnknownCommand, 0, nullptr);
}

void NNControl::Run()
{
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
	handleVerificationRequest();

	NeuralControlInput input{};
	NeuralControlOutput output{};
	bool inference_valid = buildInput(input, cycle_start);

	if (!inference_valid && _last_input_valid) {
		_network.reset();
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

		if (!_network.run(input, output)) {
			_inhibit_reason = InhibitReason::InferenceUnavailable;
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
	}

	bool authority = false;

	if (inference_valid) {
		_last_output = output;
		authority = actuatorAuthorityAllowed();
	}

	// Random placeholder weights are benchmark-only. Keep the complete
	// inference and publication workload, but never forward random outputs.
	if (inference_valid && authority && !kRandomModelDryRun) {
		_inhibit_reason = InhibitReason::None;
		publishOutput(&output, cycle_start);
		_output_enabled = true;

	} else {
		if (inference_valid && kRandomModelDryRun) {
			_inhibit_reason = InhibitReason::RandomWeightsDryRun;
		}

		++_rejected_count;
		publishOutput(nullptr, cycle_start);
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
	case InhibitReason::InvalidInput: return "non-finite input";
	case InhibitReason::InferenceUnavailable: return "model unavailable/rejected";
	case InhibitReason::InvalidOutput: return "invalid model output";
	case InhibitReason::RandomWeightsDryRun: return "random-weight dry-run";
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
	PX4_INFO("21-128-128-4 int8 GRU, target 500 Hz, measured %.1f Hz", (double)measured_rate_hz);
	PX4_INFO("mode: %s, weights: %lu bytes, checksum: 0x%08lx",
		 kRandomModelDryRun ? "RANDOM-WEIGHT DRY-RUN" : "actuator-enabled",
		 (unsigned long)NeuralNetworkInterface::weightBytes(),
		 (unsigned long)_network.weightChecksum());
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
	PX4_INFO("last model output: [%.4f %.4f %.4f %.4f]",
		 (double)_last_output.controls[0], (double)_last_output.controls[1],
		 (double)_last_output.controls[2], (double)_last_output.controls[3]);
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
	_network.reset();
	_verification_network.reset();
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
	PX4_INFO("GRU state and load counters reset");
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
Neural-network flight-controller interface.

The module runs a fixed-size 21-128-128-4 int8 GRU from fresh gyro callbacks.
The placeholder uses deterministic random weights and always remains in dry-run:
it performs and measures inference but publishes NaN actuator controls.

Set IMU_GYRO_RATEMAX to 1000 for a 500 Hz callback-limited benchmark. Use
`nn_control status` to inspect measured rate, inference/cycle latency and
deadline misses. Random weights cannot be enabled for actuator output.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nn_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset GRU hidden state and load counters");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int nn_control_main(int argc, char *argv[])
{
	return NNControl::main(argc, argv);
}
