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
 * The LQI backend computes outputs at 500 Hz but keeps actuator authority
 * inhibited until bench/HIL validation is complete.
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

/**
 * LQI model version
 *
 * 0 selects the previous nominal 13-state LQI weights (unchanged), 1 selects
 * the external-upper identified 19-state composite LQI, and 2 selects the
 * airframe-1 coaxial collective/servo LQI. The direct-controller firmware
 * defaults to model 2; the model is fixed at module start, so changing this
 * parameter requires restarting nn_control.
 *
 * @min 0
 * @max 2
 * @value 0 nominal 13-state
 * @value 1 identified 19-state composite
 * @value 2 coaxial collective/servo LQI
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_INT32(NN_LQI_MODEL, 2);

/**
 * Physical airframe selected by the identified LQI model
 *
 * The generated identified LQI payload embeds its own airframe number. When
 * NN_LQI_MODEL is 1 or 2, this value must match the embedded model or
 * actuator output stays inhibited. This prevents an identified model generated for one
 * airframe from being flown on another after flashing the wrong firmware.
 * Change the parameter before restarting nn_control when moving a controller
 * between registered airframes.
 *
 * @min 1
 * @max 999
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_INT32(NN_AIRFRAME_ID, 1);

/**
 * Ground-station manual-control submode enable
 *
 * Enables the RC-supervised ground-station input path. The path can only
 * replace the four manual axes while the RC link is valid, the vehicle is
 * armed, the configured RC flight-mode slot is selected, and a fresh
 * GS-Link control stream is present. This does not bypass
 * NN_LQI_OUTPUT_EN or any Commander/actuator safety gate.
 *
 * @min 0
 * @max 1
 * @value 0 disabled
 * @value 1 enabled
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_INT32(NN_GS_EN, 1);

/**
 * Ground-station mode switch channel
 *
 * Deprecated by the PX4 RC_MAP_FLTMODE flight-mode mapping. Retained for
 * parameter compatibility but is not used by the mode-slot gate.
 *
 * One-based physical RC channel used as the ground-station mode switch.
 * The channel is read from rc_channels before the MANUAL_CONTROL input is
 * accepted. Calibrate this channel and verify it with listener rc_channels.
 *
 * @min 1
 * @max 18
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_INT32(NN_GS_SW_CHAN, 8);

/**
 * Ground-station mode switch endpoint
 *
 * Deprecated by NN_GS_MODE_SLOT. Retained for parameter compatibility but
 * is not used by the mode-slot ground-station gate.
 *
 * Normalized RC switch threshold. A positive value selects the high endpoint
 * (channel >= threshold); a negative value selects the low endpoint
 * (channel <= threshold). The mode requires the opposite endpoint first and
 * the selected endpoint held for 100 ms after arming.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_FLOAT(NN_GS_SW_TH, 0.70f);

/**
 * Ground-station flight-mode slot
 *
 * One-based RC flight-mode slot used to request ground-station control. The
 * physical switch channel is configured with RC_MAP_FLTMODE and its slot
 * parameters are COM_FLTMODE1..6. External mode 2 maps to parameter value 101.
 *
 * @min 1
 * @max 6
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_INT32(NN_GS_MODE_SLOT, 6);

/**
 * Ground-station command timeout
 *
 * Maximum age of a GS-Link DATA_RC sample. When exceeded, control returns to
 * RC immediately and mode 6 must be deselected and selected again.
 *
 * @min 50
 * @max 1000
 * @unit ms
 * @group Direct Actuator Controller
 */
PARAM_DEFINE_INT32(NN_GS_TIMEOUT_MS, 250);
