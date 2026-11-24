#ifndef MOTOR_BENCH_H
#define MOTOR_BENCH_H

#include "stm32f4xx_hal.h"
#include "route_library.h"

void MotorBench_Init(UART_HandleTypeDef *console_uart,
                     UART_HandleTypeDef *motor_uart);
void MotorBench_Poll(void);
uint8_t MotorBench_WorkGroup(void);
void MotorBench_CompleteWork(void);
uint8_t MotorBench_StartPath(uint8_t id);
uint8_t MotorBench_StartLeg(RouteStation from,RouteStation to);
uint8_t MotorBench_Shift(int16_t right_mm);
uint8_t MotorBench_TrimShift(int16_t right_mm);
uint8_t MotorBench_TrimTravel(int16_t forward_mm);
const char *MotorBench_LivePhase(uint8_t *path,uint8_t *step,uint8_t *count);

void MotorBench_Command(const char *command);
uint8_t MotorBench_IsBusy(void);
uint8_t MotorBench_HasFault(void);
void MotorBench_Stop(const char *reason,uint8_t fault);
void MotorBench_OnRx(UART_HandleTypeDef *uart);
void MotorBench_OnError(UART_HandleTypeDef *uart);
#endif
