#ifndef EMM_BUS_H
#define EMM_BUS_H
#include "stm32f4xx_hal.h"
typedef struct {
  uint8_t busy,done,ok,raw,addr,code,expected,length,reply[64];
  uint32_t started,last_rx;
} EmmExchange;
typedef struct {
  UART_HandleTypeDef *uart;
  struct { volatile uint16_t head,tail; uint8_t data[256]; } rx;
  uint8_t byte;
  volatile uint8_t rx_error;
  uint32_t ready_at,timeout_ms;
  EmmExchange exchange;
} EmmBus;
void Emm_Init(EmmBus *,UART_HandleTypeDef *,uint32_t);
void Emm_OnRx(EmmBus *);
void Emm_OnError(EmmBus *);
void Emm_Cancel(EmmBus *);
void Emm_Start(EmmBus *,const uint8_t *,uint8_t,uint8_t,uint8_t);
void Emm_Poll(EmmBus *);
int Emm_TakeReply(EmmBus *);
void Emm_PrintRawReply(EmmBus *);
void Emm_StopBroadcast(EmmBus *);
#endif
