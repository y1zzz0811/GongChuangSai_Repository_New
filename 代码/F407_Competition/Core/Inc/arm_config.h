#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H
/* M5=42 base rotation; M6=35 lift; M7=28 linear rail.
 * These are independent driver profiles, NOT inferred from frame size.
 * Verify each driver's actual step angle and microsteps before movement.
 * Editing these macros does not write driver settings. */
#define ARM_M5_FULL_STEPS         200U
#define ARM_M6_FULL_STEPS         200U
#define ARM_M7_FULL_STEPS         200U
#define ARM_M5_MICROSTEPS         16U
#define ARM_M6_MICROSTEPS         16U
#define ARM_M7_MICROSTEPS         16U
/* Physical single-move caps: 0.001 degree for M5, 0.001 mm for M6/M7.
 * Pulse caps are recalculated from LIVE calibration, including raw move/plan. */
#define ARM_M5_MAX_TRAVEL_MILLI   180000U
#define ARM_M6_MAX_TRAVEL_MILLI   200000U
#define ARM_M7_MAX_TRAVEL_MILLI   100000U
/* Fixed RELATIVE strokes used by rotate cw/ccw, lift up/down, rail forward/backward
 * when the distance argument is omitted. These are not homing/absolute targets. */
#define ARM_M5_FIXED_TRAVEL_MILLI 90000U
#define ARM_M6_FIXED_TRAVEL_MILLI 100000U
#define ARM_M7_FIXED_TRAVEL_MILLI 50000U
#define ARM_M5_MAX_RPM            150U
#define ARM_M6_MAX_RPM            200U
#define ARM_M7_MAX_RPM            200U
/* Power-on per-axis speeds. Omitted RPM arguments use these live settings.
 * Change macros once and rebuild for new defaults; speed commands edit RAM. */
#define ARM_M5_DEFAULT_RPM        50U
#define ARM_M6_DEFAULT_RPM        150U
#define ARM_M7_DEFAULT_RPM        150U
#define ARM_ACCELERATION         10U
#define ARM_REPLY_TIMEOUT_MS     180U
#define ARM_MAX_MOVE_TIME_MS     120000U
#define ARM_POSITION_TOLERANCE   146U  /* encoder units, capped for small moves */
#define ARM_M6_ROUTE_TOLERANCE_UM 300U
#define ARM_M7_ROUTE_TOLERANCE_UM 300U
#define ARM_ROUTE_STABLE_SPAN_UM  100U
#define ARM_ROUTE_STABLE_MS      200U
#define ARM_ROUTE_RECHECK_MS    1000U
/* Start coordinates relative to the operator-recorded mechanical zero. */
#define ARM_ROUTE_RAIL_MILLI     65000U
/* Power-on physical calibration. M5/M6 retained from the user's verified setup.
 * M5: 1:1 motor-to-base ratio. M6: one motor revolution moves the belt 73 mm.
 * M7: user-confirmed 1:1 shaft-to-pinion drive, 62.830 mm rack travel/rev.
 * Keep the exact revolution ratio; do not round pulses/mm to an integer.
 * CAL_TRAVEL_MILLI is total travel in thousandths of deg/mm, not pulses/mm.
 * Set a ratio to zero only if deliberately disabling that physical axis.
 * POS_DIR maps CW / UP / FORWARD to a driver direction byte; 255 = unknown.
 * Runtime 'cal' changes live in RAM. Edit these defaults to keep calibration. */
#define ARM_M5_CAL_PULSES         (ARM_M5_FULL_STEPS * ARM_M5_MICROSTEPS)
#define ARM_M5_CAL_TRAVEL_MILLI   360000U
#define ARM_M5_POS_DIR            0U
#define ARM_M6_CAL_PULSES         (ARM_M6_FULL_STEPS * ARM_M6_MICROSTEPS)
#define ARM_M6_CAL_TRAVEL_MILLI   73000U
#define ARM_M6_POS_DIR            1U  /* measured: UP=1, DOWN=0 */
#define ARM_M7_CAL_PULSES         (ARM_M7_FULL_STEPS * ARM_M7_MICROSTEPS)
#define ARM_M7_CAL_TRAVEL_MILLI   62830U
#define ARM_M7_POS_DIR            1U  /* measured: FORWARD=1, BACKWARD=0 (2026-11-02) */
#endif
