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
#define CHASSIS_FORWARD_DIRECTIONS  {1U, 0U, 0U, 1U} /* M1 FL, M2 FR, M3 RL, M4 RR */
#define CHASSIS_DEFAULT_RPM         10U
#define CHASSIS_MAX_RPM            100U
#define CHASSIS_STRAIGHT_SCALE    1000U /* permille: 1000 = 1.000 */
#define CHASSIS_TURN_SCALE        1000U
#define CHASSIS_POSITION_TOLERANCE  146U /* Encoder counts; about 0.8 degrees. */
#define CHASSIS_REPLY_TIMEOUT_MS   180U
#define CHASSIS_SETTLE_MS          250U
#if CHASSIS_MICROSTEPS < 1 || CHASSIS_MICROSTEPS > 256
#error Invalid microstep setting
#endif
#if CHASSIS_WHEEL_DIAMETER_MM < 10 || CHASSIS_WHEEL_DIAMETER_MM > 1000
#error Invalid wheel diameter
#endif
#if CHASSIS_DEFAULT_RPM < 1 || CHASSIS_DEFAULT_RPM > CHASSIS_MAX_RPM
#error Invalid default speed
#endif
#endif
