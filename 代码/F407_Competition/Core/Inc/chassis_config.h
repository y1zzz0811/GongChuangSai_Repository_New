#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H
/* Persistent build-time defaults. L/W are wheel-center spacings, in mm.
 * CLI geometry/speed/cal commands temporarily override these defaults in RAM.
 * Fixed scope: direct-drive, 1.8-degree motors, X-layout mecanum wheels, Emm.
 */
#define CHASSIS_LENGTH_MM          190U
#define CHASSIS_WIDTH_MM           215U
#define CHASSIS_WHEEL_DIAMETER_MM   80U
#define CHASSIS_MICROSTEPS          16U
#define CHASSIS_PULSES_PER_REV      (200U * CHASSIS_MICROSTEPS)
/* Legacy C5.1 wiring-frame wheel positions, NOT the newly named body front.
 * C5.2 remaps logical translations in motor_bench.c without changing addresses.
 * Direction calibration remains BY ADDRESS. New body mounting is unconfirmed.
 */
#define CHASSIS_FRONT_LEFT_ADDR      2U
#define CHASSIS_FRONT_RIGHT_ADDR     1U
#define CHASSIS_REAR_LEFT_ADDR       3U
#define CHASSIS_REAR_RIGHT_ADDR      4U
/* C5.1 wiring reference by M1..M4, retained underneath the C5.2 logical map. */
#define CHASSIS_DIRECTION_BASE      {1U, 0U, 0U, 1U}
#define CHASSIS_FORE_AFT_REVERSED     1U /* User's joint hardware test, 2026-10-29. */
#define CHASSIS_DEFAULT_LINEAR_RPM 150U /* Forward/backward and lateral translation. */
#define CHASSIS_DEFAULT_TURN_RPM   150U /* In-place turns; wheel RPM, not body deg/s. */
#define CHASSIS_MAX_RPM             300U
#define CHASSIS_ACCELERATION        10U /* Emm 0..255 level, NOT RPM/s; preserve A4. */
#define CHASSIS_STRAIGHT_SCALE    1000U /* permille: 1000 = 1.000 */
#define CHASSIS_TURN_SCALE        1000U
#define CHASSIS_POSITION_TOLERANCE  146U /* Encoder counts; about 0.8 degrees. */
#define CHASSIS_REPLY_TIMEOUT_MS   180U
#define CHASSIS_SETTLE_MS          250U
#if CHASSIS_FRONT_LEFT_ADDR < 1 || CHASSIS_FRONT_LEFT_ADDR > 4 || CHASSIS_FRONT_RIGHT_ADDR < 1 || CHASSIS_FRONT_RIGHT_ADDR > 4 || CHASSIS_REAR_LEFT_ADDR < 1 || CHASSIS_REAR_LEFT_ADDR > 4 || CHASSIS_REAR_RIGHT_ADDR < 1 || CHASSIS_REAR_RIGHT_ADDR > 4
#error Wheel addresses must be in the range 1..4
#endif
#if CHASSIS_FRONT_LEFT_ADDR == CHASSIS_FRONT_RIGHT_ADDR || CHASSIS_FRONT_LEFT_ADDR == CHASSIS_REAR_LEFT_ADDR || CHASSIS_FRONT_LEFT_ADDR == CHASSIS_REAR_RIGHT_ADDR || CHASSIS_FRONT_RIGHT_ADDR == CHASSIS_REAR_LEFT_ADDR || CHASSIS_FRONT_RIGHT_ADDR == CHASSIS_REAR_RIGHT_ADDR || CHASSIS_REAR_LEFT_ADDR == CHASSIS_REAR_RIGHT_ADDR
#error Each physical wheel must have a different address
#endif
#if CHASSIS_MICROSTEPS < 1 || CHASSIS_MICROSTEPS > 256
#error Invalid microstep setting
#endif
#if CHASSIS_FORE_AFT_REVERSED > 1
#error Invalid fore/aft direction correction
#endif
#if CHASSIS_WHEEL_DIAMETER_MM < 10 || CHASSIS_WHEEL_DIAMETER_MM > 1000
#error Invalid wheel diameter
#endif
#if CHASSIS_MAX_RPM < 1 || CHASSIS_MAX_RPM > 65535
#error Invalid speed limit for the 16-bit protocol field
#endif
#if CHASSIS_ACCELERATION > 255
#error Invalid Emm acceleration level
#endif
#if CHASSIS_DEFAULT_LINEAR_RPM < 1 || CHASSIS_DEFAULT_LINEAR_RPM > CHASSIS_MAX_RPM
#error Invalid default linear speed
#endif
#if CHASSIS_DEFAULT_TURN_RPM < 1 || CHASSIS_DEFAULT_TURN_RPM > CHASSIS_MAX_RPM
#error Invalid default turn speed
#endif
#endif
