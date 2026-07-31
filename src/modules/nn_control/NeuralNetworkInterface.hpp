/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

/**
 * Stable input contract between PX4/uORB and the neural-network runtime.
 *
 * Coordinate conventions are the conventions of the source uORB topics:
 * quaternion is [w, x, y, z], angular velocity and acceleration are body
 * FRD, and local position/velocity are NED. Manual-control axes are [-1, 1].
 */
struct NeuralControlInput {
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
 * Four model outputs. The placeholder mapping is motor 1, motor 2, servo 1,
 * servo 2. Actuator publication remains disabled while random weights are in
 * use.
 */
struct NeuralControlOutput {
	float controls[4];
};

/**
 * Fixed-size 21-128-128-4, fully-integer GRU inference scaffold.
 *
 * Weights are generated deterministically during init() so the placeholder
 * exercises the same matrix-vector dimensions and memory traffic as a real
 * model without committing a large generated weight file. Model export will
 * eventually replace initRandomWeights() and the placeholder quantization
 * constants.
 */
class NeuralNetworkInterface
{
public:
	static constexpr int kInputSize{21};
	static constexpr int kHiddenSize{128};
	static constexpr int kOutputSize{4};
	static constexpr int kGateCount{3};
	static constexpr uint32_t kMacsPerInference{156032};

	bool init();
	bool run(const NeuralControlInput &input, NeuralControlOutput &output);
	bool runFeatures(const float (&features)[kInputSize], NeuralControlOutput &output);
	void reset();

	bool initialized() const { return _initialized; }
	uint32_t weightChecksum() const { return _weight_checksum; }
	static constexpr uint32_t weightBytes();

private:
	static constexpr int kInputPadded{24};

	void initRandomWeights();
	void quantizeInput(const NeuralControlInput &input, int8_t (&quantized)[kInputPadded]) const;
	void quantizeFeatures(const float (&features)[kInputSize], int8_t (&quantized)[kInputPadded]) const;
	void gruLayer(const int8_t *input, int input_size, const int8_t *input_weights,
		      const int8_t *recurrent_weights, const int32_t *input_bias,
		      const int32_t *recurrent_bias,
		      const int8_t *hidden_in, int8_t *hidden_out);
	static int32_t dotProduct(const int8_t *a, const int8_t *b, int length);
	static int16_t sigmoidQ15(int32_t accumulator);
	static int16_t tanhQ15(int32_t accumulator);
	static int8_t quantize(float value, float inverse_scale);

	int8_t _hidden_1[kHiddenSize]{};
	int8_t _hidden_2[kHiddenSize]{};
	uint32_t _weight_checksum{0};
	bool _initialized{false};
};

constexpr uint32_t NeuralNetworkInterface::weightBytes()
{
	return kGateCount * kHiddenSize * kInputPadded
	       + 3 * kGateCount * kHiddenSize * kHiddenSize
	       + kOutputSize * kHiddenSize
	       + (4 * kGateCount * kHiddenSize + kOutputSize) * sizeof(int32_t);
}
