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
/* Physical wheel positions, confirmed from above with the chassis front ahead.
 * The bus still uses addresses 1..4. Direction calibration remains BY ADDRESS.
 */
#define CHASSIS_FRONT_LEFT_ADDR      2U
#define CHASSIS_FRONT_RIGHT_ADDR     1U
#define CHASSIS_REAR_LEFT_ADDR       3U
#define CHASSIS_REAR_RIGHT_ADDR      4U
#define CHASSIS_FORWARD_DIRECTIONS  {1U, 0U, 0U, 1U} /* By address M1, M2, M3, M4. */
#define CHASSIS_DEFAULT_LINEAR_RPM  10U /* Forward/backward and lateral translation. */
#define CHASSIS_DEFAULT_TURN_RPM    10U /* In-place turns; wheel RPM, not body deg/s. */
#define CHASSIS_MAX_RPM            100U
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
#if CHASSIS_WHEEL_DIAMETER_MM < 10 || CHASSIS_WHEEL_DIAMETER_MM > 1000
#error Invalid wheel diameter
#endif
#if CHASSIS_MAX_RPM < 1 || CHASSIS_MAX_RPM > 65535
#error Invalid speed limit for the 16-bit protocol field
#endif
#if CHASSIS_DEFAULT_LINEAR_RPM < 1 || CHASSIS_DEFAULT_LINEAR_RPM > CHASSIS_MAX_RPM
#error Invalid default linear speed
#endif
#if CHASSIS_DEFAULT_TURN_RPM < 1 || CHASSIS_DEFAULT_TURN_RPM > CHASSIS_MAX_RPM
#error Invalid default turn speed
#endif
#endif
