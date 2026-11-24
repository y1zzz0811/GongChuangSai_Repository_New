#include "emm_bus.h"
#include "app_console.h"
#include <string.h>
#define REPLY_CAP 64U
#define MOTOR_CAP 256U
static int Due(uint32_t now,uint32_t target) { return (int32_t)(now-target)>=0; }
void Emm_Init(EmmBus *b,UART_HandleTypeDef *uart,uint32_t timeout) {
  memset(b,0,sizeof(*b)); b->uart=uart; b->timeout_ms=timeout; b->ready_at=HAL_GetTick();
  if (HAL_UART_Receive_IT(uart,&b->byte,1U)!=HAL_OK) b->rx_error=1U;
}
void Emm_OnRx(EmmBus *b) {
  uint16_t next=(uint16_t)((b->rx.head+1U)&255U);
  if (next==b->rx.tail) b->rx_error=1U;
  else { b->rx.data[b->rx.head]=b->byte; __DMB(); b->rx.head=next; }
  if (HAL_UART_Receive_IT(b->uart,&b->byte,1U)!=HAL_OK) b->rx_error=1U;
}
void Emm_OnError(EmmBus *b) {
  b->rx_error=1U; (void)HAL_UART_AbortReceive(b->uart);
  (void)HAL_UART_Receive_IT(b->uart,&b->byte,1U);
}
static void Finish(EmmBus *b,uint8_t ok) {
  b->exchange.busy=0U; b->exchange.done=1U; b->exchange.ok=ok; b->ready_at=HAL_GetTick()+3U;
}
void Emm_Cancel(EmmBus *b) {
  b->exchange.busy=b->exchange.done=0U; b->rx.tail=b->rx.head; b->rx_error=0U;
}
void Emm_Start(EmmBus *b,const uint8_t *f,uint8_t n,uint8_t expected,uint8_t raw) {
  memset(&b->exchange,0,sizeof(b->exchange)); b->exchange.busy=1U; b->exchange.raw=raw;
  b->exchange.addr=f[0]; b->exchange.code=f[1]; b->exchange.expected=expected;
  b->exchange.started=b->exchange.last_rx=HAL_GetTick(); b->rx.tail=b->rx.head; b->rx_error=0U;
  if (HAL_UART_Transmit(b->uart,(uint8_t *)f,n,10U)!=HAL_OK) Finish(b,0U);
  if (raw) App_Log("TX (%u bytes) sent.\r\n",(unsigned)n);
}
void Emm_Poll(EmmBus *b) {
  uint8_t value; uint32_t now=HAL_GetTick();
  if (!b->exchange.busy) return;
  if (b->rx_error) { b->rx_error=0U; Finish(b,0U); return; }
  while (b->exchange.busy && b->rx.tail!=b->rx.head) {
    value=b->rx.data[b->rx.tail]; __DMB();
    b->rx.tail=(uint16_t)((b->rx.tail+1U)&(MOTOR_CAP-1U)); b->exchange.last_rx=now;
    if (b->exchange.length==REPLY_CAP) {
      if (b->exchange.raw) { Finish(b,1U); break; }
      memmove(b->exchange.reply,b->exchange.reply+1,--b->exchange.length);
    }
    b->exchange.reply[b->exchange.length++]=value;
    if (b->exchange.raw) continue;
    /* Match address, function and fixed length; skip unrelated notifications. */
    while (b->exchange.length>=2U) {
      if (b->exchange.reply[0]!=b->exchange.addr || (b->exchange.reply[1]!=b->exchange.code && b->exchange.reply[1]!=0U)) {
        memmove(b->exchange.reply,b->exchange.reply+1,--b->exchange.length); continue;
      }
      if (b->exchange.length>=4U && b->exchange.reply[3]==0x6BU &&
          (b->exchange.reply[2]==0xE2U || b->exchange.reply[2]==0xEEU)) { Finish(b,0U); break; }
      if (b->exchange.reply[1]==0U) {
        if (b->exchange.length<4U) break;
        memmove(b->exchange.reply,b->exchange.reply+1,--b->exchange.length); continue;
      }
      if (b->exchange.length<b->exchange.expected) break;
      if (b->exchange.reply[b->exchange.expected-1U]==0x6BU &&
          (b->exchange.expected!=4U || b->exchange.reply[2]==2U || b->exchange.code==0x3AU)) { Finish(b,1U); break; }
      memmove(b->exchange.reply,b->exchange.reply+1,--b->exchange.length);
    }
  }
  if (!b->exchange.busy) return;
  if (b->exchange.raw && b->exchange.length && Due(now,b->exchange.last_rx+12U)) Finish(b,1U);
  else if (Due(now,b->exchange.started+(b->exchange.raw?500U:b->timeout_ms))) Finish(b,0U);
}
int Emm_TakeReply(EmmBus *b) {
  int r; if (!b->exchange.done) return 0;
  r=b->exchange.ok?1:-1; b->exchange.done=0U; return r;
}
void Emm_PrintRawReply(EmmBus *b) {
  uint8_t i; static char out[REPLY_CAP*3U+24U]; size_t used;
  if (!b->exchange.done || !b->exchange.raw) return;
  if (!b->exchange.length) App_Print("No reply; check power, mode, wiring and address.\r\n");
  else {
    used=(size_t)App_Format(out,sizeof(out),"RX (%u):",(unsigned)b->exchange.length);
    for (i=0U;i<b->exchange.length;++i) used+=(size_t)App_Format(out+used,sizeof(out)-used," %02X",b->exchange.reply[i]);
    (void)App_Format(out+used,sizeof(out)-used,"\r\n"); App_Print(out);
    if (!b->exchange.ok) App_Print("RX incomplete or UART error.\r\n");
  }
  b->exchange.done=b->exchange.raw=0U; App_Print("> ");
}
void Emm_StopBroadcast(EmmBus *b) {
  uint8_t f[]={0U,0xFEU,0x98U,0U,0x6BU};
  if (HAL_UART_Transmit(b->uart,f,sizeof(f),10U)!=HAL_OK) App_Print("STOP TX failed; use verified hardware stop.\r\n");
  b->ready_at=HAL_GetTick()+5U;
}
