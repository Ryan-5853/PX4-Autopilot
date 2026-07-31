#!/usr/bin/env python3
"""
Compare the PX4 nn_control int8 GRU against a bit-equivalent local reference.

The script uses MAVLink 2 DEBUG_FLOAT_ARRAY messages:
  NNGRU_REQ (id 42420): protocol header plus 21 float observations
  NNGRU_RSP (id 42421): status, MCU inference time, four outputs, checksum

Random weights, bias ordering, quantization, activation LUT and recurrent state
updates intentionally mirror src/modules/nn_control/NeuralNetworkInterface.cpp.
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

os.environ.setdefault("MAVLINK20", "1")

import numpy as np

try:
    from pymavlink import mavutil
except ModuleNotFoundError:
    mavutil = None
else:
    mavutil.set_dialect("common")


PROTOCOL_VERSION = 1
REQUEST_ID = 42420
RESPONSE_ID = 42421
REQUEST_NAME = b"NNGRU_REQ"
RESPONSE_NAME = "NNGRU_RSP"

COMMAND_INFO = 0
COMMAND_RESET = 1
COMMAND_STEP = 2

STATUS_NAMES = {
    0: "OK",
    1: "BAD_VERSION",
    2: "BAD_CHECKSUM",
    3: "INVALID_OBSERVATION",
    4: "INFERENCE_FAILED",
    5: "UNKNOWN_COMMAND",
}

INPUT_SIZE = 21
INPUT_PADDED = 24
HIDDEN_SIZE = 128
OUTPUT_SIZE = 4
GATE_COUNT = 3
LOGICAL_MACS = 156032

INVERSE_SCALES = np.asarray(
    [
        127, 127, 127, 127,
        32, 32, 32,
        4, 4, 4,
        8, 8, 8,
        16, 16, 16,
        127, 127, 127, 127, 127,
    ],
    dtype=np.float32,
)

# Explicit little-endian int16 Q15 table shared with the firmware. Keeping the
# bytes fixed avoids platform-specific NumPy/libm tanh rounding differences.
TANH_LUT = np.frombuffer(
    bytes.fromhex(
        "0180018001800180018001800180018001800180018001800180018001800180"
        "0180018001800180018001800180018001800180018001800180018001800180"
        "0180018002800280028002800280028002800280028003800380038003800480"
        "0480048005800580068007800780088009800a800b800d800e80108012801480"
        "17801a801d80218025802a80308036803d8045804e805880638070807f809080"
        "a380b980d180ed800c812f8157818481b881f28133827d82d18230839b831384"
        "9c843585e385a686828779888f89c78a258cad8d638f4c916c93ca956a98519b"
        "859e0ba2e7a51eaab4aeabb303b9bebedac452cb22d241d9a7e047e815f003f8"
        "0000fd07eb0fb917591fbf26de2dae34263b4241fd46554c4c51e255195af55d"
        "7b61af649667366a946cb46e9d705372db733975717687777e785a791d7acb7a"
        "647bed7b657cd07c2f7d837dcd7d0e7e487e7c7ea97ed17ef47e137f2f7f47"
        "7f5d7f707f817f907f9d7fa87fb27fbb7fc37fca7fd07fd67fdb7fdf7fe37f"
        "e67fe97fec7fee7ff07ff27ff37ff57ff67ff77ff87ff97ff97ffa7ffb7ffb"
        "7ffc7ffc7ffc7ffd7ffd7ffd7ffd7ffe7ffe7ffe7ffe7ffe7ffe7ffe7ffe7ffe"
        "7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff"
        "7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff7fff"
        "7fff7fff7fff7fff7fff7fff7fff7fff7fff7f"
    ),
    dtype="<i2",
    count=257,
)


class XorShift32:
    def __init__(self, seed: int) -> None:
        self.state = seed & 0xFFFFFFFF

    def next(self) -> int:
        state = self.state
        state ^= (state << 13) & 0xFFFFFFFF
        state ^= state >> 17
        state ^= (state << 5) & 0xFFFFFFFF
        self.state = state & 0xFFFFFFFF
        return self.state


def random_array(
    generator: XorShift32, shape: Tuple[int, ...], magnitude: int, dtype: np.dtype
) -> np.ndarray:
    count = int(np.prod(shape))
    values = np.empty(count, dtype=dtype)

    for index in range(count):
        random_value = generator.next() & 0x7FFFFFFF
        values[index] = (random_value % (2 * magnitude + 1)) - magnitude

    return values.reshape(shape)


def fnv1a(checksum: int, data: bytes) -> int:
    for value in data:
        checksum ^= value
        checksum = (checksum * 16777619) & 0xFFFFFFFF

    return checksum


def trunc_divide(values: np.ndarray, divisor: int) -> np.ndarray:
    values64 = values.astype(np.int64)
    return np.where(
        values64 >= 0,
        values64 // divisor,
        -((-values64) // divisor),
    )


@dataclass
class Response:
    status: int
    sequence: int
    inference_us: float
    outputs: np.ndarray
    checksum: int
    weight_bytes: int
    macs: int


class LocalGru:
    def __init__(self) -> None:
        generator = XorShift32(0x4E4E4752)
        rows = GATE_COUNT * HIDDEN_SIZE

        self.w1_input = random_array(
            generator, (rows, INPUT_PADDED), 8, np.int8
        )
        self.w1_recurrent = random_array(
            generator, (rows, HIDDEN_SIZE), 8, np.int8
        )
        self.w2_input = random_array(
            generator, (rows, HIDDEN_SIZE), 8, np.int8
        )
        self.w2_recurrent = random_array(
            generator, (rows, HIDDEN_SIZE), 8, np.int8
        )
        self.w_output = random_array(
            generator, (OUTPUT_SIZE, HIDDEN_SIZE), 8, np.int8
        )

        self.b1_input = random_array(generator, (rows,), 256, np.dtype("<i4"))
        self.b1_recurrent = random_array(
            generator, (rows,), 256, np.dtype("<i4")
        )
        self.b2_input = random_array(generator, (rows,), 256, np.dtype("<i4"))
        self.b2_recurrent = random_array(
            generator, (rows,), 256, np.dtype("<i4")
        )
        self.b_output = random_array(
            generator, (OUTPUT_SIZE,), 256, np.dtype("<i4")
        )

        checksum = 2166136261

        for array in (
            self.w1_input,
            self.w1_recurrent,
            self.w2_input,
            self.w2_recurrent,
            self.w_output,
            self.b1_input,
            self.b1_recurrent,
            self.b2_input,
            self.b2_recurrent,
            self.b_output,
        ):
            checksum = fnv1a(checksum, array.tobytes(order="C"))

        self.checksum = checksum
        self.weight_bytes = sum(
            array.nbytes
            for array in (
                self.w1_input,
                self.w1_recurrent,
                self.w2_input,
                self.w2_recurrent,
                self.w_output,
                self.b1_input,
                self.b1_recurrent,
                self.b2_input,
                self.b2_recurrent,
                self.b_output,
            )
        )

        self.tanh_lut = TANH_LUT

        self.hidden_1 = np.zeros(HIDDEN_SIZE, dtype=np.int8)
        self.hidden_2 = np.zeros(HIDDEN_SIZE, dtype=np.int8)

        # Matrix conversion is done once, not in the timed comparison loop.
        self.w1_input_i32 = self.w1_input.astype(np.int32)
        self.w1_recurrent_i32 = self.w1_recurrent.astype(np.int32)
        self.w2_input_i32 = self.w2_input.astype(np.int32)
        self.w2_recurrent_i32 = self.w2_recurrent.astype(np.int32)
        self.w_output_i32 = self.w_output.astype(np.int32)

    def reset(self) -> None:
        self.hidden_1.fill(0)
        self.hidden_2.fill(0)

    def sigmoid_q15(self, accumulator: np.ndarray) -> np.ndarray:
        indices = np.clip(
            128 + trunc_divide(accumulator, 512), 0, 256
        ).astype(np.int32)
        return (
            (self.tanh_lut[indices].astype(np.int32) + 32767) // 2
        ).astype(np.int32)

    def tanh_q15(self, accumulator: np.ndarray) -> np.ndarray:
        indices = np.clip(
            128 + trunc_divide(accumulator, 256), 0, 256
        ).astype(np.int32)
        return self.tanh_lut[indices].astype(np.int32)

    def gru_layer(
        self,
        input_vector: np.ndarray,
        hidden: np.ndarray,
        input_weights: np.ndarray,
        recurrent_weights: np.ndarray,
        input_bias: np.ndarray,
        recurrent_bias: np.ndarray,
    ) -> np.ndarray:
        input_i32 = input_vector.astype(np.int32)
        hidden_i32 = hidden.astype(np.int32)
        input_acc = input_weights @ input_i32 + input_bias.astype(np.int32)
        recurrent_acc = (
            recurrent_weights @ hidden_i32 + recurrent_bias.astype(np.int32)
        )

        reset = self.sigmoid_q15(
            input_acc[0:HIDDEN_SIZE] + recurrent_acc[0:HIDDEN_SIZE]
        )
        update = self.sigmoid_q15(
            input_acc[HIDDEN_SIZE : 2 * HIDDEN_SIZE]
            + recurrent_acc[HIDDEN_SIZE : 2 * HIDDEN_SIZE]
        )

        candidate_acc = input_acc[
            2 * HIDDEN_SIZE : 3 * HIDDEN_SIZE
        ].astype(np.int64) + (
            (
                recurrent_acc[2 * HIDDEN_SIZE : 3 * HIDDEN_SIZE].astype(
                    np.int64
                )
                * reset.astype(np.int64)
            )
            >> 15
        )
        candidate = self.tanh_q15(candidate_acc) >> 8
        mixed = (
            update.astype(np.int64) * hidden_i32.astype(np.int64)
            + (32767 - update.astype(np.int64))
            * candidate.astype(np.int64)
        )
        return np.clip(mixed >> 15, -127, 127).astype(np.int8)

    def step(self, observations: Sequence[float]) -> np.ndarray:
        observation_f32 = np.asarray(observations, dtype=np.float32)

        if observation_f32.shape != (INPUT_SIZE,):
            raise ValueError(f"expected {INPUT_SIZE} observations")

        quantized = np.rint(
            observation_f32 * INVERSE_SCALES
        ).clip(-127, 127).astype(np.int8)
        padded = np.zeros(INPUT_PADDED, dtype=np.int8)
        padded[:INPUT_SIZE] = quantized

        next_hidden_1 = self.gru_layer(
            padded,
            self.hidden_1,
            self.w1_input_i32,
            self.w1_recurrent_i32,
            self.b1_input,
            self.b1_recurrent,
        )
        next_hidden_2 = self.gru_layer(
            next_hidden_1,
            self.hidden_2,
            self.w2_input_i32,
            self.w2_recurrent_i32,
            self.b2_input,
            self.b2_recurrent,
        )
        self.hidden_1 = next_hidden_1
        self.hidden_2 = next_hidden_2

        output_acc = (
            self.w_output_i32 @ self.hidden_2.astype(np.int32)
            + self.b_output.astype(np.int32)
        )
        output_q15 = self.tanh_q15(output_acc)
        return (
            output_q15.astype(np.float32) / np.float32(32767.0)
        ).astype(np.float32)


class MavlinkVerifier:
    def __init__(
        self,
        connection_string: str,
        baud: int,
        timeout: float,
        response_rate: float,
    ) -> None:
        if mavutil is None:
            raise RuntimeError(
                "pymavlink is not installed; run setup_windows.bat or "
                "install the adjacent requirements-windows.txt"
            )

        self.timeout = timeout
        self.connection = mavutil.mavlink_connection(
            connection_string,
            baud=baud,
            source_system=255,
            source_component=190,
            autoreconnect=True,
        )
        print(f"Waiting for heartbeat on {connection_string} ...")
        heartbeat = self.connection.wait_heartbeat(timeout=timeout)

        if heartbeat is None:
            raise TimeoutError("no MAVLink heartbeat received")

        self.target_system = self.connection.target_system
        self.target_component = self.connection.target_component
        print(
            "Connected to "
            f"system={self.target_system} component={self.target_component}"
        )

        interval_us = int(round(1_000_000.0 / response_rate))
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(0.1)

    @staticmethod
    def _message_name(message: object) -> str:
        name = getattr(message, "name", "")

        if isinstance(name, bytes):
            return name.split(b"\0", 1)[0].decode("ascii", errors="replace")

        return str(name).split("\0", 1)[0]

    def request(
        self,
        command: int,
        sequence: int,
        checksum: int,
        observations: Optional[Sequence[float]] = None,
    ) -> Tuple[Response, float]:
        data = [0.0] * 58
        data[0] = float(PROTOCOL_VERSION)
        data[1] = float(command)
        data[2] = float(sequence)
        data[3] = float(checksum & 0xFFFF)
        data[4] = float((checksum >> 16) & 0xFFFF)

        if observations is not None:
            if len(observations) != INPUT_SIZE:
                raise ValueError(f"expected {INPUT_SIZE} observations")

            for index, value in enumerate(observations):
                data[5 + index] = float(np.float32(value))

        # Discard old responses so a delayed packet cannot satisfy this request.
        while self.connection.recv_match(
            type="DEBUG_FLOAT_ARRAY", blocking=False
        ) is not None:
            pass

        started = time.monotonic()
        self.connection.mav.debug_float_array_send(
            int(time.time_ns() // 1000),
            REQUEST_NAME,
            REQUEST_ID,
            data,
        )
        deadline = started + self.timeout

        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            message = self.connection.recv_match(
                type="DEBUG_FLOAT_ARRAY",
                blocking=True,
                timeout=remaining,
            )

            if message is None:
                break

            if (
                int(message.array_id) != RESPONSE_ID
                or self._message_name(message) != RESPONSE_NAME
            ):
                continue

            response_data = message.data
            response_sequence = int(round(response_data[2]))

            if response_sequence != sequence:
                continue

            round_trip_ms = (time.monotonic() - started) * 1000.0
            response_checksum = int(round(response_data[8])) | (
                int(round(response_data[9])) << 16
            )
            response = Response(
                status=int(round(response_data[1])),
                sequence=response_sequence,
                inference_us=float(response_data[3]),
                outputs=np.asarray(response_data[4:8], dtype=np.float32),
                checksum=response_checksum,
                weight_bytes=int(round(response_data[10])),
                macs=int(round(response_data[11])),
            )
            return response, round_trip_ms

        raise TimeoutError(
            f"timeout waiting for NNGRU_RSP sequence {sequence}; "
            "do not continue without resetting both recurrent states"
        )


def generate_observation(
    generator: np.random.Generator,
) -> np.ndarray:
    # Generate integer quantized values first, then convert to documented
    # physical feature units. This avoids values close to quantizer half steps.
    quantized = generator.integers(-100, 101, size=INPUT_SIZE, dtype=np.int16)
    return (
        quantized.astype(np.float32) / INVERSE_SCALES
    ).astype(np.float32)


def require_ok(response: Response, operation: str) -> None:
    if response.status != 0:
        status = STATUS_NAMES.get(response.status, f"STATUS_{response.status}")
        raise RuntimeError(f"{operation} rejected by MCU: {status}")


def percentile(values: np.ndarray, quantile: float) -> float:
    return float(np.percentile(values, quantile)) if len(values) else 0.0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare local and STM32H743 int8 GRU inference over MAVLink"
    )
    parser.add_argument(
        "connection",
        nargs="?",
        help="pymavlink connection, e.g. COM5 or udp:127.0.0.1:14550",
    )
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument(
        "--response-rate",
        type=float,
        default=20.0,
        help="requested DEBUG_FLOAT_ARRAY stream rate in Hz (default: 20)",
    )
    parser.add_argument("--seed", type=int, default=20260731)
    parser.add_argument("--atol", type=float, default=2e-5)
    parser.add_argument(
        "--print-every",
        type=int,
        default=10,
        help="print every N successful steps; use 1 for every step",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="verify local weights/LUT/recurrent execution without an MCU",
    )
    args = parser.parse_args()

    if not args.self_test and not args.connection:
        parser.error("connection is required unless --self-test is used")

    if args.cycles <= 0:
        parser.error("--cycles must be positive")

    if args.cycles > 16_777_214:
        parser.error("--cycles is too large for an exact float32 sequence")

    if args.response_rate <= 0:
        parser.error("--response-rate must be positive")

    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    local = LocalGru()
    print(
        f"Local weights: {local.weight_bytes} bytes, "
        f"checksum=0x{local.checksum:08x}"
    )

    if local.checksum != 0xB8EA4FEE or local.weight_bytes != 163344:
        raise RuntimeError(
            "local model self-check failed: expected checksum/bytes "
            "0xb8ea4fee/163344"
        )

    if args.self_test:
        observation_generator = np.random.default_rng(args.seed)
        output = np.zeros(OUTPUT_SIZE, dtype=np.float32)

        for _ in range(args.cycles):
            output = local.step(generate_observation(observation_generator))

        if not np.all(np.isfinite(output)):
            raise RuntimeError("local model self-check produced non-finite output")

        if args.cycles == 100 and args.seed == 20260731:
            expected_output = np.asarray(
                [
                    0.12436292320489883,
                    -0.2449110448360443,
                    -0.18533891439437866,
                    0.7615894079208374,
                ],
                dtype=np.float32,
            )

            if not np.array_equal(output, expected_output):
                raise RuntimeError(
                    "100-step local recurrent self-check output mismatch: "
                    f"expected={expected_output.tolist()}, got={output.tolist()}"
                )

        print(
            f"Local self-test PASSED: {args.cycles} recurrent steps, "
            f"last output={output.tolist()}"
        )
        return 0

    verifier = MavlinkVerifier(
        args.connection,
        args.baud,
        args.timeout,
        args.response_rate,
    )

    info, info_rtt = verifier.request(
        COMMAND_INFO, 0, local.checksum
    )

    if info.checksum != local.checksum:
        raise RuntimeError(
            "weight checksum mismatch: "
            f"local=0x{local.checksum:08x}, MCU=0x{info.checksum:08x}"
        )

    require_ok(info, "INFO")

    if info.weight_bytes != local.weight_bytes or info.macs != LOGICAL_MACS:
        raise RuntimeError(
            "model metadata mismatch: "
            f"local bytes/MACs={local.weight_bytes}/{LOGICAL_MACS}, "
            f"MCU={info.weight_bytes}/{info.macs}"
        )

    print(
        f"MCU model matched: {info.weight_bytes} bytes, "
        f"{info.macs} logical MACs/step, INFO RTT={info_rtt:.2f} ms"
    )

    reset, reset_rtt = verifier.request(
        COMMAND_RESET, 1, local.checksum
    )
    require_ok(reset, "RESET")
    local.reset()
    print(f"Both recurrent states reset, RTT={reset_rtt:.2f} ms")

    observation_generator = np.random.default_rng(args.seed)
    inference_times = []
    round_trip_times = []
    max_errors = []

    for cycle in range(args.cycles):
        sequence = cycle + 2
        observations = generate_observation(observation_generator)
        response, round_trip_ms = verifier.request(
            COMMAND_STEP,
            sequence,
            local.checksum,
            observations,
        )
        require_ok(response, f"STEP {cycle}")
        local_output = local.step(observations)
        absolute_error = np.abs(response.outputs - local_output)
        max_error = float(np.max(absolute_error))

        if not np.allclose(
            response.outputs,
            local_output,
            rtol=0.0,
            atol=args.atol,
        ):
            print(f"Mismatch at cycle {cycle}, sequence {sequence}", file=sys.stderr)
            print(f"observations={observations.tolist()}", file=sys.stderr)
            print(f"MCU output={response.outputs.tolist()}", file=sys.stderr)
            print(f"local output={local_output.tolist()}", file=sys.stderr)
            print(f"abs error={absolute_error.tolist()}", file=sys.stderr)
            print(
                "States are now intentionally considered unsynchronized; "
                "rerun the script to reset both sides.",
                file=sys.stderr,
            )
            return 2

        inference_times.append(response.inference_us)
        round_trip_times.append(round_trip_ms)
        max_errors.append(max_error)

        if (
            args.print_every > 0
            and ((cycle + 1) % args.print_every == 0 or cycle == 0)
        ):
            print(
                f"cycle={cycle + 1:5d}/{args.cycles} "
                f"MCU={response.inference_us:7.1f} us "
                f"RTT={round_trip_ms:7.2f} ms "
                f"max_abs_error={max_error:.8f}"
            )

    inference = np.asarray(inference_times, dtype=np.float64)
    round_trip = np.asarray(round_trip_times, dtype=np.float64)
    errors = np.asarray(max_errors, dtype=np.float64)

    print("\nVerification PASSED")
    print(
        "MCU inference [us]: "
        f"min={inference.min():.1f}, mean={inference.mean():.1f}, "
        f"p95={percentile(inference, 95):.1f}, "
        f"p99={percentile(inference, 99):.1f}, max={inference.max():.1f}"
    )
    print(
        "MAVLink RTT [ms]:    "
        f"min={round_trip.min():.2f}, mean={round_trip.mean():.2f}, "
        f"p95={percentile(round_trip, 95):.2f}, "
        f"max={round_trip.max():.2f}"
    )
    print(
        "Output error:        "
        f"mean={errors.mean():.9f}, max={errors.max():.9f}, "
        f"tolerance={args.atol:.9f}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted; rerun to reset both recurrent states.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
