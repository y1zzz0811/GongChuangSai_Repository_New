#ifndef ARM_BENCH_H
#define ARM_BENCH_H
#include "stm32f4xx_hal.h"
#define ARM_PICKUP_RAIL_LIMIT_MM 20
void ArmBench_Init(UART_HandleTypeDef *console_uart, UART_HandleTypeDef *motor_uart);
void ArmBench_Poll(void);
uint8_t ArmBench_RouteAction(uint8_t axis,uint8_t negative,uint32_t amount_milli,uint16_t rpm);
uint8_t ArmBench_CheckRouteStart(void);
uint8_t ArmBench_HasReference(void);
uint8_t ArmBench_RouteReady(void);
uint8_t ArmBench_RouteRail(int16_t from_mm,int16_t to_mm);
void ArmBench_ResetPickupRail(void);
uint8_t ArmBench_PickupRail(int16_t from_offset_mm,int16_t to_offset_mm);
void ArmBench_ReportReadiness(void);
void ArmBench_Command(const char *command);
uint8_t ArmBench_IsBusy(void);
uint8_t ArmBench_HasFault(void);
uint8_t ArmBench_IsArmed(void);
const char *ArmBench_LivePhase(void);
void ArmBench_Stop(const char *reason,uint8_t fault);
void ArmBench_OnRx(UART_HandleTypeDef *uart);
void ArmBench_OnError(UART_HandleTypeDef *uart);
#endif
