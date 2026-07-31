/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "NeuralNetworkInterface.hpp"

#include <cmath>
#include <cstring>

#include <mathlib/math/Functions.hpp>

namespace
{

using NN = NeuralNetworkInterface;

// These arrays intentionally live in static RAM. A real exported model can
// keep the same layout and either initialize this AXI-SRAM image from flash or
// read const weights directly after board timing has been characterized.
alignas(32) int8_t g_w1_input[NN::kGateCount * NN::kHiddenSize * 24];
alignas(32) int8_t g_w1_recurrent[NN::kGateCount * NN::kHiddenSize * NN::kHiddenSize];
alignas(32) int8_t g_w2_input[NN::kGateCount * NN::kHiddenSize * NN::kHiddenSize];
alignas(32) int8_t g_w2_recurrent[NN::kGateCount * NN::kHiddenSize * NN::kHiddenSize];
alignas(32) int8_t g_w_output[NN::kOutputSize * NN::kHiddenSize];

alignas(32) int32_t g_b1_input[NN::kGateCount * NN::kHiddenSize];
alignas(32) int32_t g_b1_recurrent[NN::kGateCount * NN::kHiddenSize];
alignas(32) int32_t g_b2_input[NN::kGateCount * NN::kHiddenSize];
alignas(32) int32_t g_b2_recurrent[NN::kGateCount * NN::kHiddenSize];
alignas(32) int32_t g_b_output[NN::kOutputSize];

// tanh over [-8, 8] in steps of 1/16, represented as signed Q15. Keep this
// table identical to the host reference so platform libm cannot affect it.
alignas(32) int16_t g_tanh_lut[257] {
	-32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767,
	-32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767,
	-32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767,
	-32767, -32767, -32767, -32767, -32767, -32767, -32767, -32767,
	-32767, -32767, -32766, -32766, -32766, -32766, -32766, -32766,
	-32766, -32766, -32766, -32765, -32765, -32765, -32765, -32764,
	-32764, -32764, -32763, -32763, -32762, -32761, -32761, -32760,
	-32759, -32758, -32757, -32755, -32754, -32752, -32750, -32748,
	-32745, -32742, -32739, -32735, -32731, -32726, -32720, -32714,
	-32707, -32699, -32690, -32680, -32669, -32656, -32641, -32624,
	-32605, -32583, -32559, -32531, -32500, -32465, -32425, -32380,
	-32328, -32270, -32205, -32131, -32047, -31952, -31845, -31725,
	-31588, -31435, -31261, -31066, -30846, -30599, -30321, -30009,
	-29659, -29267, -28829, -28340, -27796, -27190, -26518, -25775,
	-24955, -24053, -23065, -21986, -20812, -19541, -18173, -16706,
	-15142, -13486, -11742, -9919, -8025, -6073, -4075, -2045,
	0, 2045, 4075, 6073, 8025, 9919, 11742, 13486,
	15142, 16706, 18173, 19541, 20812, 21986, 23065, 24053,
	24955, 25775, 26518, 27190, 27796, 28340, 28829, 29267,
	29659, 30009, 30321, 30599, 30846, 31066, 31261, 31435,
	31588, 31725, 31845, 31952, 32047, 32131, 32205, 32270,
	32328, 32380, 32425, 32465, 32500, 32531, 32559, 32583,
	32605, 32624, 32641, 32656, 32669, 32680, 32690, 32699,
	32707, 32714, 32720, 32726, 32731, 32735, 32739, 32742,
	32745, 32748, 32750, 32752, 32754, 32755, 32757, 32758,
	32759, 32760, 32761, 32761, 32762, 32763, 32763, 32764,
	32764, 32764, 32765, 32765, 32765, 32765, 32766, 32766,
	32766, 32766, 32766, 32766, 32766, 32766, 32766, 32767,
	32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767,
	32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767,
	32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767,
	32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767,
	32767
};

uint32_t xorshift32(uint32_t &state)
{
	state ^= state << 13;
	state ^= state >> 17;
	state ^= state << 5;
	return state;
}

template<typename T, size_t N>
void fillRandom(T (&values)[N], uint32_t &state, int32_t magnitude)
{
	for (size_t i = 0; i < N; ++i) {
		const int32_t random = static_cast<int32_t>(xorshift32(state) & 0x7fffffff);
		values[i] = static_cast<T>((random % (2 * magnitude + 1)) - magnitude);
	}
}

uint32_t checksumBytes(uint32_t checksum, const void *data, size_t length)
{
	const uint8_t *bytes = static_cast<const uint8_t *>(data);

	for (size_t i = 0; i < length; ++i) {
		checksum ^= bytes[i];
		checksum *= 16777619u;
	}

	return checksum;
}

} // namespace

bool NeuralNetworkInterface::init()
{
	initRandomWeights();
	reset();
	_initialized = true;
	return true;
}

void NeuralNetworkInterface::initRandomWeights()
{
	// Fixed seed makes board measurements and host reference tests repeatable.
	uint32_t state = 0x4e4e4752u; // "NNGR"
	fillRandom(g_w1_input, state, 8);
	fillRandom(g_w1_recurrent, state, 8);
	fillRandom(g_w2_input, state, 8);
	fillRandom(g_w2_recurrent, state, 8);
	fillRandom(g_w_output, state, 8);
	fillRandom(g_b1_input, state, 256);
	fillRandom(g_b1_recurrent, state, 256);
	fillRandom(g_b2_input, state, 256);
	fillRandom(g_b2_recurrent, state, 256);
	fillRandom(g_b_output, state, 256);

	_weight_checksum = 2166136261u;
	_weight_checksum = checksumBytes(_weight_checksum, g_w1_input, sizeof(g_w1_input));
	_weight_checksum = checksumBytes(_weight_checksum, g_w1_recurrent, sizeof(g_w1_recurrent));
	_weight_checksum = checksumBytes(_weight_checksum, g_w2_input, sizeof(g_w2_input));
	_weight_checksum = checksumBytes(_weight_checksum, g_w2_recurrent, sizeof(g_w2_recurrent));
	_weight_checksum = checksumBytes(_weight_checksum, g_w_output, sizeof(g_w_output));
	_weight_checksum = checksumBytes(_weight_checksum, g_b1_input, sizeof(g_b1_input));
	_weight_checksum = checksumBytes(_weight_checksum, g_b1_recurrent, sizeof(g_b1_recurrent));
	_weight_checksum = checksumBytes(_weight_checksum, g_b2_input, sizeof(g_b2_input));
	_weight_checksum = checksumBytes(_weight_checksum, g_b2_recurrent, sizeof(g_b2_recurrent));
	_weight_checksum = checksumBytes(_weight_checksum, g_b_output, sizeof(g_b_output));
}

void NeuralNetworkInterface::reset()
{
	memset(_hidden_1, 0, sizeof(_hidden_1));
	memset(_hidden_2, 0, sizeof(_hidden_2));
}

int8_t NeuralNetworkInterface::quantize(float value, float inverse_scale)
{
	const int32_t quantized = static_cast<int32_t>(lrintf(value * inverse_scale));
	return static_cast<int8_t>(math::constrain(quantized, int32_t{-127}, int32_t{127}));
}

void NeuralNetworkInterface::quantizeInput(const NeuralControlInput &input,
		int8_t (&quantized)[kInputPadded]) const
{
	float features[kInputSize] {};
	features[0] = input.attitude_q[0];
	features[1] = input.attitude_q[1];
	features[2] = input.attitude_q[2];
	features[3] = input.attitude_q[3];

	for (int i = 0; i < 3; ++i) {
		features[4 + i] = input.angular_velocity[i];
		features[7 + i] = input.angular_acceleration[i];
		features[10 + i] = input.acceleration[i];
		features[13 + i] = input.local_velocity[i];
	}

	features[16] = input.rc_roll;
	features[17] = input.rc_pitch;
	features[18] = input.rc_yaw;
	features[19] = input.rc_throttle;
	features[20] = input.battery_remaining;
	quantizeFeatures(features, quantized);
}

void NeuralNetworkInterface::quantizeFeatures(const float (&features)[kInputSize],
		int8_t (&quantized)[kInputPadded]) const
{
	// Placeholder scales for the documented 21-feature contract. These must be
	// generated together with trained weights for the production model.
	static constexpr float inverse_scales[kInputSize] {
		127.f, 127.f, 127.f, 127.f,
		32.f, 32.f, 32.f,
		4.f, 4.f, 4.f,
		8.f, 8.f, 8.f,
		16.f, 16.f, 16.f,
		127.f, 127.f, 127.f, 127.f, 127.f
	};

	for (int i = 0; i < kInputSize; ++i) {
		quantized[i] = quantize(features[i], inverse_scales[i]);
	}

	quantized[21] = 0;
	quantized[22] = 0;
	quantized[23] = 0;
}

int32_t NeuralNetworkInterface::dotProduct(const int8_t *a, const int8_t *b, int length)
{
	int32_t accumulator = 0;
	int i = 0;

#if defined(__ARM_FEATURE_DSP) || defined(__ARM_ARCH_7EM__)
	// Four signed int8 MACs per iteration. SXTB16 extracts bytes 0/2; rotating
	// by eight bits extracts bytes 1/3. SMLAD then performs two signed 16-bit
	// products and accumulates them.
	for (; i + 3 < length; i += 4) {
		uint32_t a_bytes;
		uint32_t b_bytes;
		memcpy(&a_bytes, &a[i], sizeof(a_bytes));
		memcpy(&b_bytes, &b[i], sizeof(b_bytes));

		uint32_t a_02;
		uint32_t a_13;
		uint32_t b_02;
		uint32_t b_13;
		asm("sxtb16 %0, %1" : "=r"(a_02) : "r"(a_bytes));
		asm("sxtb16 %0, %1, ror #8" : "=r"(a_13) : "r"(a_bytes));
		asm("sxtb16 %0, %1" : "=r"(b_02) : "r"(b_bytes));
		asm("sxtb16 %0, %1, ror #8" : "=r"(b_13) : "r"(b_bytes));
		asm("smlad %0, %1, %2, %0" : "+r"(accumulator) : "r"(a_02), "r"(b_02));
		asm("smlad %0, %1, %2, %0" : "+r"(accumulator) : "r"(a_13), "r"(b_13));
	}
#endif

	for (; i < length; ++i) {
		accumulator += static_cast<int32_t>(a[i]) * static_cast<int32_t>(b[i]);
	}

	return accumulator;
}

int16_t NeuralNetworkInterface::sigmoidQ15(int32_t accumulator)
{
	// sigmoid(x) = (tanh(x / 2) + 1) / 2. Accumulator scale is a
	// placeholder chosen to keep the random model away from saturation.
	const int32_t index = math::constrain(int32_t{128} + accumulator / int32_t{512},
					      int32_t{0}, int32_t{256});
	return static_cast<int16_t>((static_cast<int32_t>(g_tanh_lut[index]) + 32767) / 2);
}

int16_t NeuralNetworkInterface::tanhQ15(int32_t accumulator)
{
	const int32_t index = math::constrain(int32_t{128} + accumulator / int32_t{256},
					      int32_t{0}, int32_t{256});
	return g_tanh_lut[index];
}

void NeuralNetworkInterface::gruLayer(const int8_t *input, int input_size,
				      const int8_t *input_weights,
				      const int8_t *recurrent_weights,
				      const int32_t *input_bias,
				      const int32_t *recurrent_bias,
				      const int8_t *hidden_in,
				      int8_t *hidden_out)
{
	const int reset_offset = 0;
	const int update_offset = kHiddenSize;
	const int candidate_offset = 2 * kHiddenSize;

	for (int row = 0; row < kHiddenSize; ++row) {
		const int reset_row = reset_offset + row;
		const int update_row = update_offset + row;
		const int candidate_row = candidate_offset + row;

		const int32_t reset_acc =
			dotProduct(input, &input_weights[reset_row * input_size], input_size)
			+ dotProduct(hidden_in, &recurrent_weights[reset_row * kHiddenSize], kHiddenSize)
			+ input_bias[reset_row] + recurrent_bias[reset_row];
		const int32_t update_acc =
			dotProduct(input, &input_weights[update_row * input_size], input_size)
			+ dotProduct(hidden_in, &recurrent_weights[update_row * kHiddenSize], kHiddenSize)
			+ input_bias[update_row] + recurrent_bias[update_row];

		const int16_t reset = sigmoidQ15(reset_acc);
		const int16_t update = sigmoidQ15(update_acc);
		const int32_t candidate_input =
			dotProduct(input, &input_weights[candidate_row * input_size], input_size)
			+ input_bias[candidate_row];
		const int32_t candidate_recurrent =
			dotProduct(hidden_in, &recurrent_weights[candidate_row * kHiddenSize], kHiddenSize)
			+ recurrent_bias[candidate_row];
		const int32_t candidate_acc =
			candidate_input + static_cast<int32_t>(
				(static_cast<int64_t>(candidate_recurrent) * reset) >> 15);
		const int32_t candidate = tanhQ15(candidate_acc) >> 8;
		const int32_t mixed = update * hidden_in[row] + (32767 - update) * candidate;
		hidden_out[row] = static_cast<int8_t>(
					 math::constrain(mixed >> 15, int32_t{-127}, int32_t{127}));
	}
}

bool NeuralNetworkInterface::run(const NeuralControlInput &input, NeuralControlOutput &output)
{
	if (!_initialized) {
		return false;
	}

	alignas(4) int8_t quantized_input[kInputPadded];
	quantizeInput(input, quantized_input);

	alignas(4) int8_t next_hidden_1[kHiddenSize];
	alignas(4) int8_t next_hidden_2[kHiddenSize];

	gruLayer(quantized_input, kInputPadded, g_w1_input, g_w1_recurrent,
		 g_b1_input, g_b1_recurrent,
		 _hidden_1, next_hidden_1);
	gruLayer(next_hidden_1, kHiddenSize, g_w2_input, g_w2_recurrent,
		 g_b2_input, g_b2_recurrent,
		 _hidden_2, next_hidden_2);

	memcpy(_hidden_1, next_hidden_1, sizeof(_hidden_1));
	memcpy(_hidden_2, next_hidden_2, sizeof(_hidden_2));

	for (int row = 0; row < kOutputSize; ++row) {
		const int32_t accumulator =
			dotProduct(_hidden_2, &g_w_output[row * kHiddenSize], kHiddenSize)
			+ g_b_output[row];
		output.controls[row] = static_cast<float>(tanhQ15(accumulator)) / 32767.f;
	}

	return true;
}

bool NeuralNetworkInterface::runFeatures(const float (&features)[kInputSize],
		NeuralControlOutput &output)
{
	if (!_initialized) {
		return false;
	}

	alignas(4) int8_t quantized_input[kInputPadded];
	quantizeFeatures(features, quantized_input);
	alignas(4) int8_t next_hidden_1[kHiddenSize];
	alignas(4) int8_t next_hidden_2[kHiddenSize];

	gruLayer(quantized_input, kInputPadded, g_w1_input, g_w1_recurrent,
		 g_b1_input, g_b1_recurrent,
		 _hidden_1, next_hidden_1);
	gruLayer(next_hidden_1, kHiddenSize, g_w2_input, g_w2_recurrent,
		 g_b2_input, g_b2_recurrent,
		 _hidden_2, next_hidden_2);

	memcpy(_hidden_1, next_hidden_1, sizeof(_hidden_1));
	memcpy(_hidden_2, next_hidden_2, sizeof(_hidden_2));

	for (int row = 0; row < kOutputSize; ++row) {
		const int32_t accumulator =
			dotProduct(_hidden_2, &g_w_output[row * kHiddenSize], kHiddenSize)
			+ g_b_output[row];
		output.controls[row] = static_cast<float>(tanhQ15(accumulator)) / 32767.f;
	}

	return true;
}
