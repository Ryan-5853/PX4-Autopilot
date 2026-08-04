/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "ControllerBackend.hpp"
#include "NNVerificationProtocol.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_servos_trim.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class NNControl : public ModuleBase<NNControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	NNControl();
	~NNControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;
	void resetDebug();

private:
	enum class InhibitReason : uint8_t {
		None = 0,
		Disarmed,
		ActuatorLockdown,
		Failsafe,
		StaleAttitude,
		StaleAngularVelocity,
		StaleAcceleration,
		StaleManualControl,
		InvalidInput,
		BackendUnavailable,
		InvalidOutput,
		BackendOutputDisabled,
	};

	void Run() override;
	void updateSubscriptions();
	void updateParameters(bool force = false);
	bool buildInput(ControllerInput &input, hrt_abstime now);
	bool actuatorAuthorityAllowed();
	bool validateOutput(const ActuatorSetpoint &output);
	void publishOutput(const ActuatorSetpoint *output, hrt_abstime now);
	void handleVerificationRequest();
	void publishVerificationResponse(uint32_t sequence, nn_verification::Status status,
					 hrt_abstime inference_time,
					 const float *output);
	static const char *reasonString(InhibitReason reason);

	static constexpr hrt_abstime kRunInterval{2_ms};
	static constexpr hrt_abstime kEstimatorTimeout{100_ms};
	static constexpr hrt_abstime kManualControlTimeout{200_ms};
	static constexpr hrt_abstime kInferenceBudget{1_ms};
	static constexpr hrt_abstime kCycleBudget{1500_us};
	static constexpr hrt_abstime kHardDeadline{2_ms};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_servos_trim_s> _actuator_servos_trim_pub{ORB_ID(actuator_servos_trim)};
	uORB::Publication<debug_array_s> _verification_response_pub{ORB_ID(debug_array)};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _verification_request_sub{ORB_ID(debug_array)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	actuator_armed_s _actuator_armed{};
	battery_status_s _battery_status{};
	failsafe_flags_s _failsafe_flags{};
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_acceleration_s _vehicle_acceleration{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_local_position_s _vehicle_local_position{};

	ControllerBackend _backend{};
	InhibitReason _inhibit_reason{InhibitReason::Disarmed};
	uint32_t _inference_count{0};
	uint32_t _rejected_count{0};
	uint32_t _inference_overruns{0};
	uint32_t _cycle_overruns{0};
	uint32_t _hard_deadline_misses{0};
	uint32_t _verification_requests{0};
	uint32_t _verification_failures{0};
	hrt_abstime _last_verification_time{0};
	hrt_abstime _max_verification_time{0};
	hrt_abstime _last_inference_time{0};
	hrt_abstime _max_inference_time{0};
	hrt_abstime _last_cycle_time{0};
	hrt_abstime _max_cycle_time{0};
	hrt_abstime _last_sample_timestamp{0};
	ActuatorSetpoint _last_output{};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _inference_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": inference")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _budget_overrun_perf{perf_alloc(PC_COUNT, MODULE_NAME": 1.5ms budget overrun")};
	perf_counter_t _deadline_miss_perf{perf_alloc(PC_COUNT, MODULE_NAME": 2ms deadline miss")};
	perf_counter_t _verification_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": MAVLink verification")};
	bool _output_enabled{false};
	bool _last_input_valid{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NN_SV_TRIM1>) _param_servo_trim_1,
		(ParamFloat<px4::params::NN_SV_TRIM2>) _param_servo_trim_2,
		(ParamFloat<px4::params::NN_SV_TRIM3>) _param_servo_trim_3
	)
};
