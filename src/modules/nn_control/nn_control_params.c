/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <parameters/param.h>

/**
 * Servo 1 normalized trim
 *
 * Added to Servo 1 after the selected controller backend. For a 1000-2000 us
 * output range, 0.02 corresponds to approximately 10 us.
 *
 * @min -0.5
 * @max 0.5
 * @decimal 3
 * @increment 0.001
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_FLOAT(NN_SV_TRIM1, 0.0f);

/**
 * Servo 2 normalized trim
 *
 * Added to Servo 2 after the selected controller backend.
 *
 * @min -0.5
 * @max 0.5
 * @decimal 3
 * @increment 0.001
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_FLOAT(NN_SV_TRIM2, 0.0f);

/**
 * Servo 3 normalized trim
 *
 * Added to Servo 3 after the selected controller backend.
 *
 * @min -0.5
 * @max 0.5
 * @decimal 3
 * @increment 0.001
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_FLOAT(NN_SV_TRIM3, 0.0f);

/**
 * LQI backend actuator output enable
 *
 * The Plan A external-collective LQI backend computes outputs at 500 Hz but
 * keeps actuator authority inhibited until bench/HIL validation is complete.
 * Set to 1 only after the validated controller has passed the bench and HIL
 * acceptance criteria; firmware output stays NaN while this is 0.
 *
 * @min 0
 * @max 1
 * @value 0 disabled
 * @value 1 enabled
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_INT32(NN_LQI_OUTPUT_EN, 0);
