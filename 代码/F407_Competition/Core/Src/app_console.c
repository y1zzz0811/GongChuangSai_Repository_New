#include "app_console.h"
#include "competition.h"
#include "motor_bench.h"
#include "arm_bench.h"
#include "qr_bridge.h"
#include <stdarg.h>
#include <string.h>
static UART_HandleTypeDef *console;
static struct { volatile uint16_t head,tail; uint8_t data[512]; } rx;
static uint8_t byte,line_len,discard;
static volatile uint8_t urgent,error;
static char line[160];
static uint8_t telemetry;
uint8_t AppConsole_Telemetry(void) { return telemetry; }
void AppConsole_SetTelemetry(uint8_t enabled) { telemetry=(uint8_t)(enabled!=0U); }

int App_ParseU32(const char *s,uint32_t low,uint32_t high,uint32_t *out) {
  uint32_t value=0U;
  if (!s || !*s || !out) return 0;
  while (*s) {
    uint32_t digit;
    if (*s<'0' || *s>'9') return 0;
    digit=(uint32_t)(*s++-'0');
    if (value>high/10U || (value==high/10U && digit>high%10U)) return 0;
    value=value*10U+digit;
  }
  if (value<low) return 0;
  *out=value; return 1;
}
int App_ParseHexByte(const char *s,uint8_t *out) {
  unsigned i,value=0U;
  if (!s || !out || strlen(s)!=2U) return 0;
  for (i=0U;i<2U;++i) {
    unsigned char c=(unsigned char)s[i]; unsigned digit;
    if (c>='0' && c<='9') digit=c-'0';
    else if (c>='a' && c<='f') digit=c-'a'+10U;
    else if (c>='A' && c<='F') digit=c-'A'+10U;
    else return 0;
    value=(value<<4)|digit;
  }
  *out=(uint8_t)value; return 1;
}

static void Put(char *out,size_t cap,size_t *used,char c) {
  if (*used+1U<cap) out[*used]=c;
  ++*used;
}
static int Format(char *out,size_t cap,const char *fmt,va_list ap) {
  size_t used=0U;
  while (*fmt) {
    char digits[32],pad=' ',spec; unsigned width=0U,n=0U; uint8_t lng=0U,negative=0U;
    unsigned long value;
    if (*fmt!='%') { Put(out,cap,&used,*fmt++); continue; }
    ++fmt;
    if (*fmt=='0') { pad='0'; ++fmt; }
    while (*fmt>='0' && *fmt<='9') { if (width<100U) width=width*10U+(unsigned)(*fmt-'0'); ++fmt; }
    if (*fmt=='l') { lng=1U; ++fmt; }
    spec=*fmt; if (!spec) break; ++fmt;
    if (spec=='s') {
      const char *s=va_arg(ap,const char *); if (!s) s="(null)";
      while (*s) Put(out,cap,&used,*s++);
      continue;
    }
    if (spec=='c') { Put(out,cap,&used,(char)va_arg(ap,int)); continue; }
    if (spec=='%') { Put(out,cap,&used,'%'); continue; }
    if (spec=='d' || spec=='i') {
      long v=lng?va_arg(ap,long):(long)va_arg(ap,int);
      negative=(uint8_t)(v<0); value=negative?0UL-(unsigned long)v:(unsigned long)v;
    } else if (spec=='u' || spec=='x' || spec=='X') value=lng?va_arg(ap,unsigned long):(unsigned long)va_arg(ap,unsigned);
    else { Put(out,cap,&used,'?'); continue; }
    do {
      unsigned base=(spec=='x' || spec=='X')?16U:10U;
      unsigned d=(unsigned)(value%base); value/=base;
      digits[n++]=(char)(d<10U?'0'+d:(spec=='X'?'A':'a')+d-10U);
    } while (value);
    if (negative && pad=='0') Put(out,cap,&used,'-');
    while (width>n+negative) { Put(out,cap,&used,pad); --width; }
    if (negative && pad!='0') Put(out,cap,&used,'-');
    while (n) Put(out,cap,&used,digits[--n]);
  }
  if (cap) out[used<cap?used:cap-1U]='\0';
  return (int)used;
}
int App_Format(char *out,size_t cap,const char *fmt,...) {
  int result; va_list ap; va_start(ap,fmt); result=Format(out,cap,fmt,ap); va_end(ap); return result;
}
void App_Print(const char *s) {
  if (console) (void)HAL_UART_Transmit(console,(uint8_t *)s,(uint16_t)strlen(s),100U);
}
void App_Log(const char *fmt,...) {
  char out[256]; va_list ap; va_start(ap,fmt); (void)Format(out,sizeof(out),fmt,ap); va_end(ap); App_Print(out);
}
void AppConsole_Init(UART_HandleTypeDef *uart) {
  telemetry=1U;
  console=uart; memset(&rx,0,sizeof(rx)); urgent=error=line_len=discard=0U;
  if (HAL_UART_Receive_IT(console,&byte,1U)!=HAL_OK) error=1U;
}
void AppConsole_ClearInput(void) { rx.tail=rx.head; line_len=discard=0U; }
uint8_t AppConsole_StopPending(void) { return (uint8_t)(urgent || error); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *u) {
  if (u==console) {
    uint16_t next=(uint16_t)((rx.head+1U)&511U);
    if (byte=='!' || byte==3U) urgent=1U;
    else if (next==rx.tail) error=1U;
    else { rx.data[rx.head]=byte; __DMB(); rx.head=next; }
    if (HAL_UART_Receive_IT(console,&byte,1U)!=HAL_OK) error=1U;
  } else { MotorBench_OnRx(u); ArmBench_OnRx(u); QrBridge_OnRx(u); }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *u) {
  if (u==console) {
    error=1U; (void)HAL_UART_AbortReceive(u);
    if (HAL_UART_Receive_IT(u,&byte,1U)!=HAL_OK) error=1U;
  } else { MotorBench_OnError(u); ArmBench_OnError(u); QrBridge_OnError(u); }
}
void AppConsole_Poll(void) {
  uint8_t budget=96U;
  if (urgent || error) {
    uint8_t bad=error; urgent=error=0U; AppConsole_ClearInput();
    Competition_Stop(bad?"console RX error/overflow":"immediate stop",bad); return;
  }
  while (budget-- && rx.tail!=rx.head) {
    uint8_t c=rx.data[rx.tail]; __DMB(); rx.tail=(uint16_t)((rx.tail+1U)&511U);
    if (c=='\r' || c=='\n') {
      if (discard) { App_Print("Line too long; discarded.\r\n"); discard=line_len=0U; }
      else if (line_len) { line[line_len]='\0'; Competition_Command(line); line_len=0U; App_Print("> "); }
    } else if (c=='\b' || c==127U) { if (line_len) --line_len; }
    else if (!discard) { if (line_len>=sizeof(line)-1U) discard=1U; else line[line_len++]=(char)c; }
    if (urgent || error) return;
  }
}
