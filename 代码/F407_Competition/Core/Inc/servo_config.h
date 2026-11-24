#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H
/* Measured state presets; changes require rebuilding. Raw pulse debugging
 * does not change these presets. S1 raw debugging and ramps use the user-authorized 850us ceiling.
 * State commands use the measured open/close presets.
 * Nominal 270deg / 2000us is ONLY the unchanged speed scale for both servos. */
#define SERVO_NOMINAL_RANGE_DEG 270U
#define SERVO_NOMINAL_SPAN_US 2000U
#define SERVO_CLAW_OPEN_US 500U
#define SERVO_CLAW_CLOSE_US 800U
#define SERVO_CLAW_HARD_MAX_US 850U
#define SERVO_TRAY_1_US 850U
#define SERVO_TRAY_2_US 1500U
#define SERVO_TRAY_3_US 2080U
#if SERVO_CLAW_OPEN_US < 500 || SERVO_CLAW_CLOSE_US <= SERVO_CLAW_OPEN_US || SERVO_CLAW_CLOSE_US > SERVO_CLAW_HARD_MAX_US || SERVO_CLAW_HARD_MAX_US > 850
#error Invalid measured claw pulse limits
#endif
#if SERVO_TRAY_1_US < 500 || SERVO_TRAY_1_US >= SERVO_TRAY_2_US || SERVO_TRAY_2_US >= SERVO_TRAY_3_US || SERVO_TRAY_3_US > 2500
#error Invalid measured tray pulse states
#endif
#if SERVO_NOMINAL_RANGE_DEG < 1 || SERVO_NOMINAL_RANGE_DEG > 360 || SERVO_NOMINAL_SPAN_US < 1 || SERVO_NOMINAL_SPAN_US > 2000
#error Invalid nominal servo speed scale
#endif
#define SERVO_S1_SPEED_DPS 90U
#define SERVO_S2_SPEED_DPS 90U
#endif
