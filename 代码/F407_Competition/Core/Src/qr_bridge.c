#include "qr_bridge.h"
#include "app_console.h"
#include "mission.h"
#include <string.h>
static UART_HandleTypeDef *pi_uart,*hmi_uart;
static struct { volatile uint16_t head,tail; uint8_t data[256]; } rx;
static volatile uint8_t rx_fault;
static uint8_t byte,len,discard,dirty,valid,tx_ok,locked;
static char line[80],slots[4][4],task[16];
static uint32_t next_tx,received_at,sequence,accepted,rejected,rx_errors,tx_errors;
static uint32_t request_id,requested_at,ring_received_at,ring_age;
typedef enum { TARGET_RING, TARGET_TOP, TARGET_RAW } VisionTarget;
static uint8_t wanted_ring,ring_ready;
static VisionTarget wanted_target;
static const char *const target_tags[]={"","TOP","RAW"};
static int32_t ring_offset[2];
static uint32_t vision_bytes,vision_frames,vision_bad;
static const char *vision_reason;
static void VisionReject(const char *reason) {
  if(!vision_bad) App_Log("VISION rejected: %s\r\n",reason);
  ++vision_bad; vision_reason=reason;
}
#define RING_TIMEOUT_MS 12000U
#define TOP_TIMEOUT_MS 9000U
#define RAW_TIMEOUT_MS 90000U

static uint32_t VisionTimeout(void) {
  return wanted_target==TARGET_TOP?TOP_TIMEOUT_MS:
    wanted_target==TARGET_RAW?RAW_TIMEOUT_MS:RING_TIMEOUT_MS;
}

static uint32_t VisionMaxAge(void) {
  return wanted_target==TARGET_TOP?3000U:1500U;
}

void QrBridge_CancelRing(void) { wanted_ring=ring_ready=0U; }

static uint8_t RequestTarget(uint8_t ring,VisionTarget target,int16_t forward_mm,int16_t left_mm,
                             uint8_t follow_up) {
  char request[32];
  int n;
  QrBridge_CancelRing();
  if (!pi_uart || ring<1U || ring>(target==TARGET_RAW?6U:3U) || AppConsole_StopPending()) return 0U;
  if (++request_id==0U) ++request_id;
  requested_at=HAL_GetTick();
  wanted_ring=ring;
  wanted_target=target;
  vision_bytes=vision_frames=vision_bad=0U; vision_reason="no_VR";
  if (target==TARGET_RAW && (follow_up || forward_mm || left_mm))
    n=App_Format(request,sizeof(request),"VQ,%lu,%u,RAW,%d,%d\n",
                 (unsigned long)request_id,ring,(int)forward_mm,(int)left_mm);
  else
    n=App_Format(request,sizeof(request),"VQ,%lu,%u%s%s\n",(unsigned long)request_id,ring,
                 target==TARGET_RING?"":",",target_tags[target]);
  if (n<0 || n>=(int)sizeof(request)) { QrBridge_CancelRing(); return 0U; }
  if (HAL_UART_Transmit(pi_uart,(uint8_t *)request,(uint16_t)n,10U)==HAL_OK) {
    App_Log("VISION request id=%lu kind=%s target=%u; timeout=%lu ms\r\n",
      (unsigned long)request_id,target==TARGET_RING?"RING":target_tags[target],ring,(unsigned long)VisionTimeout());
    return 1U;
  }
  App_Print("VISION request UART TX failed.\r\n");
  QrBridge_CancelRing();
  return 0U;
}
uint8_t QrBridge_RequestRing(uint8_t ring) { return RequestTarget(ring,TARGET_RING,0,0,0U); }
uint8_t QrBridge_RequestTop(uint8_t ring) { return RequestTarget(ring,TARGET_TOP,0,0,0U); }
uint8_t QrBridge_RequestRaw(uint8_t color,int16_t forward_mm,int16_t left_mm,uint8_t follow_up) {
  return RequestTarget(color,TARGET_RAW,forward_mm,left_mm,follow_up);
}

int QrBridge_ReadRing(int32_t *forward,int32_t *left) {
  if (!wanted_ring || HAL_GetTick()-requested_at>=VisionTimeout()) {
    if(wanted_ring) App_Log("VISION timeout id=%lu RXbytes=%lu VR=%lu rejected=%lu last=%s\r\n",
      (unsigned long)request_id,(unsigned long)vision_bytes,(unsigned long)vision_frames,
      (unsigned long)vision_bad,vision_reason);
    QrBridge_CancelRing();
    return -1;
  }
  if (!ring_ready) return 0;
  if (HAL_GetTick()-ring_received_at+ring_age>VisionMaxAge() ||
      (wanted_target==TARGET_RAW && HAL_GetTick()-ring_received_at>150U)) {
    App_Print("VISION accepted reply expired before use.\r\n");
    QrBridge_CancelRing();
    return -1;
  }
  *forward=ring_offset[0]; *left=ring_offset[1];
  QrBridge_CancelRing();
  return 1;
}

static uint8_t SignedOffset(const char *s,int32_t *out) {
  uint32_t magnitude;
  uint8_t negative=(uint8_t)(*s=='-');
  if (!App_ParseU32(s+negative,0U,500000U,&magnitude)) return 0U;
  *out=negative?-(int32_t)magnitude:(int32_t)magnitude;
  return 1U;
}

static void AcceptRing(void) {
  uint8_t crc=0U,received;
  char *fields[7],*cursor=line+3,*end=strrchr(line,',');
  uint32_t id,ring,frame,age;
  unsigned i,j,count=wanted_target==TARGET_RING?6U:7U;
  if (!wanted_ring || ring_ready) return;
  ++vision_frames;
  if (HAL_GetTick()-requested_at>=VisionTimeout()) { VisionReject("late"); return; }
  if (!end || end<=cursor || strlen(end+1)!=2U || !App_ParseHexByte(end+1,&received)) { VisionReject("format"); return; }
  for (char *p=line;p<end;++p) {
    crc^=(uint8_t)*p;
    for(j=0;j<8U;++j) crc=(uint8_t)((crc<<1U)^((crc&128U)?7U:0U));
  }
  if (crc!=received) { VisionReject("crc"); return; }
  *end='\0';
  for (i=0;i<count;++i) {
    char *comma=strchr(cursor,',');
    if ((comma!=NULL)!=(i+1U<count)) { VisionReject("fields_or_target_type"); return; }
    fields[i]=cursor;
    if (comma) { *comma='\0'; cursor=comma+1; }
  }
  if(wanted_target!=TARGET_RING && strcmp(fields[6],target_tags[wanted_target])) { VisionReject("target_type"); return; }
  if(!App_ParseU32(fields[0],1U,UINT32_MAX,&id) || id!=request_id) { VisionReject("request_id"); return; }
  if(!App_ParseU32(fields[1],1U,wanted_target==TARGET_RAW?6U:3U,&ring) || ring!=wanted_ring) { VisionReject("target_id"); return; }
  if(!App_ParseU32(fields[2],1U,UINT32_MAX,&frame)) { VisionReject("frame"); return; }
  if(!App_ParseU32(fields[3],0U,VisionMaxAge(),&age)) { VisionReject("age"); return; }
  if(!SignedOffset(fields[4],&ring_offset[0]) || !SignedOffset(fields[5],&ring_offset[1])) { VisionReject("offset_units_or_range"); return; }
  ring_age=age; ring_received_at=HAL_GetTick(); ring_ready=1U;
  App_Log("VISION accepted id=%lu fwd=%ld left=%ld um\r\n",(unsigned long)id,(long)ring_offset[0],(long)ring_offset[1]);
}

void QrBridge_Clear(void) {
  unsigned i;
  uint32_t irq=__get_PRIMASK();
  __disable_irq(); rx.tail=rx.head; __set_PRIMASK(irq);
  discard=0U; /* discard cached/queued input at a new scan boundary */
  QrBridge_CancelRing();
  for (i=0U;i<4U;++i) memcpy(slots[i],"---",4U);
  locked=valid=tx_ok=len=0U; task[0]='\0'; dirty=1U;
}
void QrBridge_Init(UART_HandleTypeDef *pi,UART_HandleTypeDef *hmi) {
  pi_uart=pi; hmi_uart=hmi; memset(&rx,0,sizeof(rx));
  request_id=HAL_GetTick();
  rx_fault=discard=0U; received_at=sequence=accepted=rejected=rx_errors=tx_errors=0U;
  QrBridge_Clear(); next_tx=HAL_GetTick()+1000U;
  if (pi && HAL_UART_Receive_IT(pi,&byte,1U)!=HAL_OK) rx_fault=1U;
}
void QrBridge_OnRx(UART_HandleTypeDef *uart) {
  uint16_t next;
  if (uart!=pi_uart) return;
  next=(uint16_t)((rx.head+1U)&255U);
  if (next==rx.tail) rx_fault=1U;
  else { rx.data[rx.head]=byte; __DMB(); rx.head=next; }
  if (HAL_UART_Receive_IT(uart,&byte,1U)!=HAL_OK) rx_fault=1U;
}
void QrBridge_OnError(UART_HandleTypeDef *uart) {
  if (uart!=pi_uart) return;
  rx_fault=1U; (void)HAL_UART_AbortReceive(uart);
  (void)HAL_UART_Receive_IT(uart,&byte,1U);
}
static void AcceptLine(void) {
  MissionCode parsed; unsigned i;
  if (len>=3U && !memcmp(line,"VR,",3U)) { AcceptRing(); return; }
  if (locked) return;
  if (len!=18U || memcmp(line,"QR,",3U) || !Mission_ParseCode(line+3,&parsed)) goto invalid;
  for (i=0U;i<4U;++i) { memcpy(slots[i],line+3U+i*4U,3U); slots[i][3]='\0'; }
  ++accepted; dirty=1U; tx_ok=0U;
  memcpy(task,parsed.text,sizeof(task)); valid=1U; received_at=HAL_GetTick(); ++sequence;
  App_Log("QR received: %s\r\n",task);
  return;
invalid:
  ++rejected; App_Print("QR invalid; unchanged.\r\n");
}
void QrBridge_Resend(void) { dirty=1U; tx_ok=0U; }
void QrBridge_LockCode(void) { if (valid) locked=1U; }
void QrBridge_Poll(void) {
  unsigned budget=64U;
  if (!pi_uart || !hmi_uart) return;
  if (rx_fault) {
    uint32_t irq=__get_PRIMASK();
    __disable_irq(); rx.tail=rx.head; rx_fault=0U; __set_PRIMASK(irq);
    ++rx_errors;
    QrBridge_CancelRing();
    if (!locked) QrBridge_Clear();
    discard=1U;
    App_Print("QR RX error; send LF then retry.\r\n");
  }
  while (budget-- && rx.tail!=rx.head) {
    uint8_t c=rx.data[rx.tail]; __DMB(); rx.tail=(uint16_t)((rx.tail+1U)&255U);
    if(wanted_ring) ++vision_bytes;
    if (discard) { if (c=='\n') discard=0U; continue; }
    if (c=='\n') {
      if (len && line[len-1U]=='\r') --len;
      if (len) { line[len]='\0'; AcceptLine(); }
      len=0U;
    } else if ((c<32U && c!='\r') || c>126U || len>=sizeof(line)-1U) {
      len=0U; discard=1U; ++rejected;
    } else line[len++]=(char)c;
    if (rx_fault || AppConsole_StopPending()) return;
  }
  if (dirty && (int32_t)(HAL_GetTick()-next_tx)>=0 && !AppConsole_StopPending()) {
    char commands[64]; int n;
    n=App_Format(commands,sizeof(commands),"t0.txt=\"%s+%s+\"%c%c%ct1.txt=\"%s+%s\"%c%c%c",
      slots[0],slots[1],255,255,255,slots[2],slots[3],255,255,255);
    if (n>0 && n<(int)sizeof(commands) &&
        HAL_UART_Transmit(hmi_uart,(uint8_t *)commands,(uint16_t)n,10U)==HAL_OK) {
      dirty=0U; tx_ok=1U;
    } else { ++tx_errors; next_tx=HAL_GetTick()+100U; }
  }
}
uint8_t QrBridge_GetCode(char out[16],uint32_t *timestamp,uint32_t *seq) {
  if (!valid || !out) return 0U;
  memcpy(out,task,sizeof(task)); if (timestamp) *timestamp=received_at; if (seq) *seq=sequence;
  return 1U;
}
void QrBridge_ShowStatus(void) {
  App_Log("QR valid=%u HMI-TX=%u seq=%lu ok=%lu bad=%lu RXerr=%lu TXerr=%lu\r\n",
    (unsigned)valid,(unsigned)tx_ok,(unsigned long)sequence,(unsigned long)accepted,
    (unsigned long)rejected,(unsigned long)rx_errors,(unsigned long)tx_errors);
}
