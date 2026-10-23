#ifndef MOTOR_BENCH_H
#define MOTOR_BENCH_H

#include "stm32f4xx_hal.h"

void MotorBench_Init(UART_HandleTypeDef *console_uart,
                     UART_HandleTypeDef *motor_uart);
void MotorBench_Poll(void);

#endif
