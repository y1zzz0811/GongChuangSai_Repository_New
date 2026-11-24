#ifndef SERVO_H
#define SERVO_H
#include "stm32f4xx_hal.h"
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U
/* TIM4 clock=84MHz, PSC=83, ARR=19999; PWM1 CH1/CH2 on PB6/PB7.
 * Finite presets plus bounded raw debugging, no position feedback. */
void Servo_Init(TIM_HandleTypeDef *timer);
void Servo_Stop(void);
uint8_t Servo_HasFault(void);
uint8_t Servo_IsBusy(void); /* output ramp pending, NOT physical arrival */
uint16_t Servo_OutputUs(uint8_t channel); /* index 0=S1, 1=S2 */
void Servo_Poll(void);
uint8_t Servo_ClawReady(void);
uint8_t Servo_Claw(uint8_t close);
uint8_t Servo_Tray(uint8_t slot);
/* Does not check vehicle interlocks; competition dispatcher owns those.
 * Both channels must have an explicit unloaded setup reference. Unknown starts
 * after boot/stop are rejected, never inferred or jumped to a target. PWM ramps
 * are command limits, NOT actual speed/position or torque feedback.
 * S1 must equal configured open/close and stay below the 800us ceiling;
 * S2 must equal one of its three measured pulses. Intermediate ramp outputs
 * are allowed between endpoints. Any invalid input rejects both targets. */
uint8_t Servo_Set(uint16_t first_us,uint16_t second_us);
void Servo_Command(const char *command);
void Servo_ShowStatus(void);
#endif
