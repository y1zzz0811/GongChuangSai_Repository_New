#ifndef QR_BRIDGE_H
#define QR_BRIDGE_H
#include "stm32f4xx_hal.h"
#define QR_UART_BAUD 115200U
#define HMI_UART_BAUD 115200U
/* Complete-frame protocol only: QR,AAA+BBB+CCC+DDD LF (CRLF accepted).
 * t0/t1 must already exist on the screen. UART TX success is NOT HMI ACK. */
void QrBridge_Init(UART_HandleTypeDef *pi,UART_HandleTypeDef *hmi);
void QrBridge_OnRx(UART_HandleTypeDef *uart);
void QrBridge_OnError(UART_HandleTypeDef *uart);
void QrBridge_Poll(void);
void QrBridge_Clear(void);
void QrBridge_Resend(void);
void QrBridge_LockCode(void);
void QrBridge_ShowStatus(void);
/* Returns only a completely validated task; does not start/advance a mission.
 * Caller uses timestamp/sequence to reject stale or already consumed scans. */
uint8_t QrBridge_GetCode(char out[16],uint32_t *timestamp,uint32_t *sequence);
uint8_t QrBridge_RequestRing(uint8_t ring);
uint8_t QrBridge_RequestTop(uint8_t ring);
uint8_t QrBridge_RequestRaw(uint8_t color,int16_t forward_mm,int16_t left_mm,uint8_t follow_up);
int QrBridge_ReadRing(int32_t *forward,int32_t *left);
void QrBridge_CancelRing(void);
#endif
