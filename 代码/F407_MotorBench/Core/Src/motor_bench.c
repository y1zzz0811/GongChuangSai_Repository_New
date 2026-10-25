#include "motor_bench.h"
#include "chassis_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#define LINE_CAP 160U
#define FRAME_CAP 32U
#define REPLY_CAP 64U
#define CONSOLE_CAP 512U
#define MOTOR_CAP 256U
typedef struct { volatile uint16_t head,tail; uint8_t data[CONSOLE_CAP]; } ConsoleRing;
typedef struct { volatile uint16_t head,tail; uint8_t data[MOTOR_CAP]; } MotorRing;
typedef struct {
  uint8_t busy,done,ok,raw,addr,code,expected,length,reply[REPLY_CAP];
  uint32_t started,last_rx;
} Exchange;
typedef enum { IDLE,PREFLIGHT_STOP,CONFIG,OPTIONS,MOTOR_ENABLE,BASELINE,LOAD,TRIGGER,
               WAIT_STATE,WAIT_POSITION,PAUSE,ABORTING,FAULT } Phase;
typedef enum { FORWARD,BACKWARD,LEFT,RIGHT,TURN_LEFT,TURN_RIGHT } Action;
typedef struct {
  Phase phase; Action action;
  uint8_t square,step,count,index,arrived,stable,abort_fault,stop_failed,flags[4];
  uint32_t distance,angle,pulses,rpm,deadline,next_at;
  int64_t baseline[4],position[4];
  char reason[80];
} Route;
static UART_HandleTypeDef *console,*motor;
static ConsoleRing console_rx;
static MotorRing motor_rx;
static uint8_t console_byte,motor_byte,address,line_length,discard_line;
static volatile uint8_t urgent_stop,console_error,motor_error;
static char line[LINE_CAP];
static Exchange exchange;
static Route route;
static uint32_t bus_ready,length_mm,width_mm,linear_rpm,turn_rpm,straight_scale,turn_scale;
static const uint8_t forward_direction[4]=CHASSIS_FORWARD_DIRECTIONS;
static int Due(uint32_t now,uint32_t target) { return (int32_t)(now-target)>=0; }
static void Print(const char *s) { (void)HAL_UART_Transmit(console,(uint8_t *)s,(uint16_t)strlen(s),200U); }
static void Log(const char *format,...) {
  static char out[256]; va_list args;
  va_start(args,format); (void)vsnprintf(out,sizeof(out),format,args); va_end(args); Print(out);
}
static uint8_t Busy(void) { return route.phase!=IDLE && route.phase!=FAULT; }
static const char *PhaseName(Phase p) {
  static const char *n[]={"idle","preflight-stop","config","options","enable","baseline",
    "preload","trigger","wait-state","wait-position","pause","stopping","fault"}; return n[(unsigned)p];
}
static const char *ActionName(Action a) {
  static const char *n[]={"forward","backward","left","right","turn-left","turn-right"}; return n[(unsigned)a];
}
/* ISR work is limited to buffering and rearming single-byte receives. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *u) {
  uint16_t next;
  if (u==console) {
    if (console_byte=='!' || console_byte==3U) urgent_stop=1U;
    else {
      next=(uint16_t)((console_rx.head+1U)&(CONSOLE_CAP-1U));
      if (next==console_rx.tail) console_error=1U;
      else { console_rx.data[console_rx.head]=console_byte; __DMB(); console_rx.head=next; }
    }
    if (HAL_UART_Receive_IT(console,&console_byte,1U)!=HAL_OK) console_error=1U;
  } else if (u==motor) {
    next=(uint16_t)((motor_rx.head+1U)&(MOTOR_CAP-1U));
    if (next==motor_rx.tail) motor_error=1U;
    else { motor_rx.data[motor_rx.head]=motor_byte; __DMB(); motor_rx.head=next; }
    if (HAL_UART_Receive_IT(motor,&motor_byte,1U)!=HAL_OK) motor_error=1U;
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *u) {
  if (u==console || u==motor) {
    if (u==console) console_error=1U; else motor_error=1U;
    (void)HAL_UART_AbortReceive(u);
    (void)HAL_UART_Receive_IT(u,u==console?&console_byte:&motor_byte,1U);
  }
}
static void ExchangeFinish(uint8_t ok) {
  exchange.busy=0U; exchange.done=1U; exchange.ok=ok; bus_ready=HAL_GetTick()+3U;
}
static void ExchangeCancel(void) {
  exchange.busy=exchange.done=0U; motor_rx.tail=motor_rx.head; motor_error=0U;
}
static void ExchangeStart(const uint8_t *f,uint8_t n,uint8_t expected,uint8_t raw) {
  memset(&exchange,0,sizeof(exchange)); exchange.busy=1U; exchange.raw=raw;
  exchange.addr=f[0]; exchange.code=f[1]; exchange.expected=expected;
  exchange.started=exchange.last_rx=HAL_GetTick(); motor_rx.tail=motor_rx.head; motor_error=0U;
  if (HAL_UART_Transmit(motor,(uint8_t *)f,n,10U)!=HAL_OK) ExchangeFinish(0U);
  if (raw) Log("TX (%u bytes) sent.\r\n",(unsigned)n);
}
static void ExchangePoll(void) {
  uint8_t value; uint32_t now=HAL_GetTick();
  if (!exchange.busy) return;
  if (motor_error) { motor_error=0U; ExchangeFinish(0U); return; }
  while (exchange.busy && motor_rx.tail!=motor_rx.head) {
    value=motor_rx.data[motor_rx.tail]; __DMB();
    motor_rx.tail=(uint16_t)((motor_rx.tail+1U)&(MOTOR_CAP-1U)); exchange.last_rx=now;
    if (exchange.length==REPLY_CAP) {
      if (exchange.raw) { ExchangeFinish(1U); break; }
      memmove(exchange.reply,exchange.reply+1,--exchange.length);
    }
    exchange.reply[exchange.length++]=value;
    if (exchange.raw) continue;
    /* Match address, function and fixed length; skip unrelated notifications. */
    while (exchange.length>=2U) {
      if (exchange.reply[0]!=exchange.addr || (exchange.reply[1]!=exchange.code && exchange.reply[1]!=0U)) {
        memmove(exchange.reply,exchange.reply+1,--exchange.length); continue;
      }
      if (exchange.length>=4U && exchange.reply[3]==0x6BU &&
          (exchange.reply[2]==0xE2U || exchange.reply[2]==0xEEU)) { ExchangeFinish(0U); break; }
      if (exchange.reply[1]==0U) {
        if (exchange.length<4U) break;
        memmove(exchange.reply,exchange.reply+1,--exchange.length); continue;
      }
      if (exchange.length<exchange.expected) break;
      if (exchange.reply[exchange.expected-1U]==0x6BU &&
          (exchange.expected!=4U || exchange.reply[2]==2U || exchange.code==0x3AU)) { ExchangeFinish(1U); break; }
      memmove(exchange.reply,exchange.reply+1,--exchange.length);
    }
  }
  if (!exchange.busy) return;
  if (exchange.raw && exchange.length && Due(now,exchange.last_rx+12U)) ExchangeFinish(1U);
  else if (Due(now,exchange.started+(exchange.raw?500U:CHASSIS_REPLY_TIMEOUT_MS))) ExchangeFinish(0U);
}
static int TakeReply(void) {
  int r; if (!exchange.done) return 0;
  r=exchange.ok?1:-1; exchange.done=0U; return r;
}
static void PrintRawReply(void) {
  uint8_t i; static char out[REPLY_CAP*3U+24U]; size_t used;
  if (!exchange.done || !exchange.raw) return;
  if (!exchange.length) Print("No reply; check power, mode, wiring and address.\r\n");
  else {
    used=(size_t)snprintf(out,sizeof(out),"RX (%u):",(unsigned)exchange.length);
    for (i=0U;i<exchange.length;++i) used+=(size_t)snprintf(out+used,sizeof(out)-used," %02X",exchange.reply[i]);
    (void)snprintf(out+used,sizeof(out)-used,"\r\n"); Print(out);
    if (!exchange.ok) Print("RX incomplete or UART error.\r\n");
  }
  exchange.done=exchange.raw=0U; Print("> ");
}
static void SendStopBroadcast(void) {
  uint8_t f[]={0U,0xFEU,0x98U,0U,0x6BU};
  if (HAL_UART_Transmit(motor,f,sizeof(f),10U)!=HAL_OK) Print("STOP transmit failed; cut motor power if needed.\r\n");
  bus_ready=HAL_GetTick()+5U;
}
static void AbortRoute(const char *reason,uint8_t fault) {
  uint8_t verify;
  fault = fault || route.phase==FAULT || (route.phase==ABORTING && route.abort_fault);
  verify=Busy() || fault;
  ExchangeCancel(); SendStopBroadcast(); (void)snprintf(route.reason,sizeof(route.reason),"%s",reason);
  route.abort_fault=fault; route.stop_failed=route.index=0U; route.phase=verify?ABORTING:IDLE;
  Log("STOP: %s%s\r\n",reason,verify?"; confirming M1..M4":"; broadcast sent");
}
static void Fail(const char *reason) {
  Log("FAULT at %s M%u code=%02X: %s\r\n",PhaseName(route.phase),
      (unsigned)(route.index+1U),exchange.code,reason);
  AbortRoute(reason,1U);
}
static int64_t DecodePosition(void) {
  uint32_t m=((uint32_t)exchange.reply[3]<<24)|((uint32_t)exchange.reply[4]<<16)|
             ((uint32_t)exchange.reply[5]<<8)|exchange.reply[6];
  return exchange.reply[2]?-(int64_t)m:(int64_t)m;
}
static uint32_t DistancePulses(uint32_t mm) {
  uint64_t den=(uint64_t)CHASSIS_WHEEL_DIAMETER_MM*3141593U*1000U;
  return (uint32_t)(((uint64_t)mm*CHASSIS_PULSES_PER_REV*1000000U*straight_scale+den/2U)/den);
}
static uint32_t TurnPulses(uint32_t deg) {
  uint64_t den=(uint64_t)360U*CHASSIS_WHEEL_DIAMETER_MM*1000U;
  return (uint32_t)(((uint64_t)CHASSIS_PULSES_PER_REV*(length_mm+width_mm)*deg*turn_scale+den/2U)/den);
}
static uint8_t Direction(uint8_t i) {
  uint8_t addr=(uint8_t)(i+1U),rev=0U;
  switch (route.action) {
    case FORWARD: break; case BACKWARD: rev=1U; break;
    case LEFT: rev=addr==CHASSIS_FRONT_LEFT_ADDR || addr==CHASSIS_REAR_RIGHT_ADDR; break;
    case RIGHT: rev=addr==CHASSIS_FRONT_RIGHT_ADDR || addr==CHASSIS_REAR_LEFT_ADDR; break;
    case TURN_RIGHT: rev=addr==CHASSIS_FRONT_RIGHT_ADDR || addr==CHASSIS_REAR_RIGHT_ADDR; break;
    case TURN_LEFT: rev=addr==CHASSIS_FRONT_LEFT_ADDR || addr==CHASSIS_REAR_LEFT_ADDR; break;
  }
  return forward_direction[i]^rev;
}
static void BeginStep(void) {
  uint64_t nominal;
  if (route.square) route.action=(route.step&1U)?TURN_RIGHT:FORWARD;
  route.rpm=route.action>=TURN_LEFT?turn_rpm:linear_rpm;
  route.pulses=route.action>=TURN_LEFT?TurnPulses(route.angle):DistancePulses(route.distance);
  nominal=(uint64_t)route.pulses*60000U/(route.rpm*CHASSIS_PULSES_PER_REV);
  route.deadline=HAL_GetTick()+(uint32_t)(nominal*3U+10000U);
  route.phase=BASELINE; route.index=route.arrived=route.stable=0U;
  Log("STEP %u/%u: %s, pulses=%lu, rpm=%lu\r\n",(unsigned)(route.step+1U),
      (unsigned)route.count,ActionName(route.action),(unsigned long)route.pulses,(unsigned long)route.rpm);
}
static uint8_t DisplacementMatches(uint8_t i) {
  int64_t delta=route.position[i]-route.baseline[i];
  uint64_t expected=((uint64_t)route.pulses*65536U+CHASSIS_PULSES_PER_REV/2U)/CHASSIS_PULSES_PER_REV;
  uint64_t actual=(uint64_t)(delta<0?-delta:delta),tol=expected/4U;
  if (tol>CHASSIS_POSITION_TOLERANCE) tol=CHASSIS_POSITION_TOLERANCE;
  if (tol<2U) tol=2U;
  return actual>=expected-tol && actual<=expected+tol;
}
static void RoutePoll(void) {
  uint32_t now=HAL_GetTick(); uint8_t f[13]; int result;
  if (!Busy()) return;
  if (route.phase!=ABORTING && Due(now,route.deadline)) { Fail("step timeout"); return; }
  if (route.phase==PAUSE) {
    if (!Due(now,route.next_at)) return;
    if (++route.step==route.count) { route.phase=IDLE; Print("DONE: route reached; check ground position/heading.\r\n> "); }
    else { BeginStep(); }
    return;
  }
  if (exchange.done) {
    result=TakeReply();
    if (route.phase==ABORTING) {
      if (result<0) route.stop_failed=1U;
      if (++route.index==4U) {
        route.phase=(route.abort_fault || route.stop_failed)?FAULT:IDLE;
        Print(route.stop_failed?"Stop confirmation incomplete; inspect motors.\r\n> ":"All four stop commands confirmed.\r\n> ");
      } return;
    }
    if (result<0) { Fail("missing/invalid reply or rejected command"); return; }
    switch (route.phase) {
      case PREFLIGHT_STOP: if (++route.index==4U) { route.index=0U; route.phase=CONFIG; } break;
      case CONFIG:
        if (exchange.reply[2]!=0x21U || exchange.reply[3]!=0x15U || exchange.reply[4]!=0x19U ||
            exchange.reply[6]!=2U || exchange.reply[9]!=(uint8_t)CHASSIS_MICROSTEPS || exchange.reply[21]!=0U ||
            (exchange.reply[22]!=1U && exchange.reply[22]!=3U)) { Fail("need Emm/1.8deg/configured microsteps/UART/6B/Receive or Both"); return; }
        if (++route.index==4U) { route.index=0U; route.phase=OPTIONS; } break;
      case OPTIONS: {
        uint16_t bits=((uint16_t)exchange.reply[2]<<8)|exchange.reply[3];
        if ((bits&0x87U)!=6U) { Fail("need closed-loop Emm, 1.8deg, speed scaling off"); return; }
        if (++route.index==4U) { route.index=0U; route.phase=MOTOR_ENABLE; } break;
      }
      case MOTOR_ENABLE: if (++route.index==4U) BeginStep(); break;
      case BASELINE:
        if (exchange.reply[2]>1U) { Fail("invalid position sign"); return; }
        route.baseline[route.index]=DecodePosition();
        if (++route.index==4U) { route.index=0U; route.phase=LOAD; } break;
      case LOAD: if (++route.index==4U) { route.index=0U; route.phase=TRIGGER; } break;
      case WAIT_STATE:
        route.flags[route.index]=exchange.reply[2];
        if (!(exchange.reply[2]&1U) || (exchange.reply[2]&0x0CU)) { Fail("motor disabled or stall flag"); return; }
        route.phase=WAIT_POSITION; break;
      case WAIT_POSITION:
        if (exchange.reply[2]>1U) { Fail("invalid position sign"); return; }
        route.position[route.index]=DecodePosition();
        if ((route.flags[route.index]&2U) && DisplacementMatches(route.index)) route.arrived|=(uint8_t)(1U<<route.index);
        if (++route.index==4U) {
          route.index=0U; if (route.arrived==15U) ++route.stable; else route.stable=0U; route.arrived=0U;
          if (route.stable>=2U) { route.phase=PAUSE; route.next_at=now+CHASSIS_SETTLE_MS;
            Print("Reached: all four positions verified.\r\n"); return; }
        }
        route.phase=WAIT_STATE; route.next_at=now+25U; break;
      default: break;
    } return;
  }
  if (exchange.busy || !Due(now,bus_ready)) return;
  if ((route.phase==WAIT_STATE || route.phase==WAIT_POSITION) && !Due(now,route.next_at)) return;
  f[0]=(uint8_t)(route.index+1U); f[2]=0x6BU;
  switch (route.phase) {
    case PREFLIGHT_STOP: case ABORTING: f[1]=0xFEU; f[2]=0x98U; f[3]=0U; f[4]=0x6BU; ExchangeStart(f,5U,4U,0U); break;
    case CONFIG: f[1]=0x42U; f[2]=0x6CU; f[3]=0x6BU; ExchangeStart(f,4U,33U,0U); break;
    case OPTIONS: f[1]=0x1AU; ExchangeStart(f,3U,5U,0U); break;
    case MOTOR_ENABLE: f[1]=0xF3U; f[2]=0xABU; f[3]=1U; f[4]=0U; f[5]=0x6BU; ExchangeStart(f,6U,4U,0U); break;
    case BASELINE: case WAIT_POSITION: f[1]=0x36U; ExchangeStart(f,3U,8U,0U); break;
    case LOAD:
      f[1]=0xFDU; f[2]=Direction(route.index); f[3]=(uint8_t)(route.rpm>>8); f[4]=(uint8_t)route.rpm; f[5]=10U;
      f[6]=(uint8_t)(route.pulses>>24); f[7]=(uint8_t)(route.pulses>>16); f[8]=(uint8_t)(route.pulses>>8); f[9]=(uint8_t)route.pulses;
      f[10]=2U; f[11]=1U; f[12]=0x6BU; ExchangeStart(f,13U,4U,0U); break;
    case TRIGGER:
      f[0]=0U; f[1]=0xFFU; f[2]=0x66U; f[3]=0x6BU;
      if (HAL_UART_Transmit(motor,f,4U,10U)!=HAL_OK) { Fail("trigger transmit failed"); return; }
      route.phase=WAIT_STATE; route.next_at=now+100U; bus_ready=now+3U; break;
    case WAIT_STATE: f[1]=0x3AU; ExchangeStart(f,3U,4U,0U); break;
    default: break;
  }
}
static int Number(const char *s,uint32_t low,uint32_t high,uint32_t *out) {
  char *end; unsigned long n;
  if (!s || *s<'0' || *s>'9') return 0;
  n=strtoul(s,&end,10); if (*end || n<low || n>high) return 0; *out=(uint32_t)n; return 1;
}
static void ShowStatus(void) {
  unsigned step=route.count?(route.step<route.count?route.step+1U:route.count):0U;
  Log("state=%s step=%u/%u action=%s target=%lu pulses\r\n",PhaseName(route.phase),step,
      (unsigned)route.count,ActionName(route.action),(unsigned long)route.pulses);
  Log("geometry L=%lu W=%lu mm; wheel=%umm/%u pulses; linear-rpm=%lu turn-rpm=%lu; cal straight=%lu turn=%lu permille\r\n",
      (unsigned long)length_mm,(unsigned long)width_mm,(unsigned)CHASSIS_WHEEL_DIAMETER_MM,
      (unsigned)CHASSIS_PULSES_PER_REV,(unsigned long)linear_rpm,(unsigned long)turn_rpm,
      (unsigned long)straight_scale,(unsigned long)turn_scale);
  Log("wheel addresses FL=%u FR=%u RL=%u RR=%u\r\n",(unsigned)CHASSIS_FRONT_LEFT_ADDR,
      (unsigned)CHASSIS_FRONT_RIGHT_ADDR,(unsigned)CHASSIS_REAR_LEFT_ADDR,(unsigned)CHASSIS_REAR_RIGHT_ADDR);
  if (route.rpm) Log("last/current step rpm=%lu\r\n",(unsigned long)route.rpm);
  Log("last flags: M1=%02X M2=%02X M3=%02X M4=%02X; selected id=%u\r\n",
      route.flags[0],route.flags[1],route.flags[2],route.flags[3],address);
  if (route.reason[0]) Log("last stop/fault: %s\r\n",route.reason);
}
static void StartRoute(Action a,uint32_t value,uint8_t square) {
  if (route.phase==FAULT) { Print("Fault latched. Inspect motors, then clearfault.\r\n"); return; }
  if (Busy() || exchange.busy || exchange.done) { Print("Busy. Use status or stopall.\r\n"); return; }
  if ((square || a>=TURN_LEFT) && (!length_mm || !width_mm)) { Print("Set measured geometry first: geometry L_mm W_mm\r\n"); return; }
  memset(&route,0,sizeof(route)); route.phase=PREFLIGHT_STOP; route.action=a; route.square=square;
  route.count=square?8U:1U; route.distance=a<TURN_LEFT?value:0U; route.angle=square?90U:value;
  route.deadline=HAL_GetTick()+15000U;
  Print("START: checking M1..M4, then enabling. Use stopall or ! to abort.\r\n");
}
static void RawTransmit(char *args) {
  uint8_t f[FRAME_CAP],n=0U; char *t=strtok(args," \t");
  while (t) {
    if (n==FRAME_CAP || strlen(t)!=2U || strspn(t,"0123456789abcdefABCDEF")!=2U) {
      Print("Use: tx 01 36 6B (2 hex digits per byte, max 32).\r\n"); return;
    }
    f[n++]=(uint8_t)strtoul(t,NULL,16); t=strtok(NULL," \t");
  }
  if (n<3U) { Print("Frame too short.\r\n"); return; } ExchangeStart(f,n,0U,1U);
}
static void ProcessLine(void) {
  char *cmd,*a,*b,*extra; uint32_t v,w; uint8_t f[5];
  if (!strcmp(line,"stopall")) { AbortRoute("operator stopall",0U); return; }
  if (!strcmp(line,"stop") && Busy()) { AbortRoute("operator stop cancelled route",0U); return; }
  if (!strcmp(line,"status")) { ShowStatus(); return; }
  if (!strcmp(line,"help")) {
    Print("help | status | id N | readpos | stop | tx HH HH ...\r\n");
    Log("geometry L_mm W_mm | speed [linear|turn] 1..%u | cal straight|turn 100..5000\r\n",(unsigned)CHASSIS_MAX_RPM);
    Print("forward|backward|left|right 10..3000(mm) | turn left|right 1..360(deg)\r\n");
    Print("square 10..3000(mm) | start (=square 1000) | stopall | ! | clearfault\r\n");
    Print("Motion uses M1..M4 on USART2. Header defaults persist; CLI overrides are RAM-only.\r\n"); return;
  }
  if (Busy() || exchange.busy || exchange.done) { Print("Busy. Use status or stopall.\r\n"); return; }
  if (!strncmp(line,"tx ",3U)) { RawTransmit(line+3); return; }
  if (!strcmp(line,"readpos") || !strcmp(line,"stop")) {
    f[0]=address;
    if (line[0]=='r') { f[1]=0x36U; f[2]=0x6BU; ExchangeStart(f,3U,0U,1U); }
    else { f[1]=0xFEU; f[2]=0x98U; f[3]=0U; f[4]=0x6BU; ExchangeStart(f,5U,0U,1U); } return;
  }
  cmd=strtok(line," \t"); a=strtok(NULL," \t"); b=strtok(NULL," \t"); extra=strtok(NULL," \t");
  if (!cmd) return;
  if (!strcmp(cmd,"id") && !b && Number(a,1U,255U,&v)) { address=(uint8_t)v; Log("Local address: %u\r\n",address); }
  else if (!strcmp(cmd,"geometry")) {
    if (!a) Log("geometry L=%lu W=%lu mm\r\n",(unsigned long)length_mm,(unsigned long)width_mm);
    else if (!extra && Number(a,10U,3000U,&v) && Number(b,10U,3000U,&w)) {
      length_mm=v; width_mm=w; Log("Geometry set (RAM). 90deg initial pulses=%lu\r\n",(unsigned long)TurnPulses(90U));
    } else Print("Use: geometry L_mm W_mm (10..3000 each)\r\n");
  } else if (!strcmp(cmd,"speed")) {
    if (!b && Number(a,1U,CHASSIS_MAX_RPM,&v)) {
      linear_rpm=turn_rpm=v; /* Preserve the legacy global-speed command. */
    } else if (!extra && a && b && Number(b,1U,CHASSIS_MAX_RPM,&v) &&
               (!strcmp(a,"linear") || !strcmp(a,"turn"))) {
      if (!strcmp(a,"linear")) linear_rpm=v; else turn_rpm=v;
    } else {
      Log("Use: speed [linear|turn] RPM (1..%u)\r\n",(unsigned)CHASSIS_MAX_RPM); return;
    }
    Log("Speed (RAM): linear=%lu RPM; turn=%lu RPM\r\n",(unsigned long)linear_rpm,(unsigned long)turn_rpm);
  }
  else if (!strcmp(cmd,"cal") && a && !extra && Number(b,100U,5000U,&v)) {
    if (!strcmp(a,"straight")) straight_scale=v; else if (!strcmp(a,"turn")) turn_scale=v;
    else { Print("Use: cal straight|turn 100..5000 (1000=1.0)\r\n"); return; } Print("Calibration set (RAM only).\r\n");
  } else if (!strcmp(cmd,"turn") && a && !extra && Number(b,1U,360U,&v) && (!strcmp(a,"left") || !strcmp(a,"right"))) {
    StartRoute(!strcmp(a,"left")?TURN_LEFT:TURN_RIGHT,v,0U);
  } else if ((!strcmp(cmd,"start") || !strcmp(cmd,"\xE5\x90\xAF\xE5\x8A\xA8")) && !a) StartRoute(FORWARD,1000U,1U);
  else if (!strcmp(cmd,"square") && !b && Number(a,10U,3000U,&v)) StartRoute(FORWARD,v,1U);
  else if (!b && Number(a,10U,3000U,&v) && (!strcmp(cmd,"forward") || !strcmp(cmd,"backward") || !strcmp(cmd,"left") || !strcmp(cmd,"right"))) {
    Action action=!strcmp(cmd,"forward")?FORWARD:!strcmp(cmd,"backward")?BACKWARD:!strcmp(cmd,"left")?LEFT:RIGHT;
    StartRoute(action,v,0U);
  } else if (!strcmp(cmd,"clearfault") && !a) {
    if (route.phase==FAULT) { route.phase=IDLE; Print("Fault cleared; next route rechecks all four motors.\r\n"); }
    else Print("No latched fault.\r\n");
  } else Print("Unknown command or invalid arguments. Type help.\r\n");
}
void MotorBench_Init(UART_HandleTypeDef *c,UART_HandleTypeDef *m) {
  console=c; motor=m; memset(&console_rx,0,sizeof(console_rx)); memset(&motor_rx,0,sizeof(motor_rx));
  memset(&exchange,0,sizeof(exchange)); memset(&route,0,sizeof(route));
  address=1U; line_length=discard_line=urgent_stop=console_error=motor_error=0U;
  length_mm=CHASSIS_LENGTH_MM; width_mm=CHASSIS_WIDTH_MM; linear_rpm=CHASSIS_DEFAULT_LINEAR_RPM; turn_rpm=CHASSIS_DEFAULT_TURN_RPM;
  straight_scale=CHASSIS_STRAIGHT_SCALE; turn_scale=CHASSIS_TURN_SCALE; bus_ready=HAL_GetTick();
  Print("F407 MotorBench A3: USART1 console, USART2 M1..M4.\r\nNo automatic movement. Type help.\r\n> ");
  if (HAL_UART_Receive_IT(console,&console_byte,1U)!=HAL_OK || HAL_UART_Receive_IT(motor,&motor_byte,1U)!=HAL_OK) console_error=1U;
}
void MotorBench_Poll(void) {
  uint8_t byte,budget=96U;
  if (urgent_stop || console_error) {
    uint8_t e=console_error; urgent_stop=console_error=0U;
    console_rx.tail=console_rx.head; line_length=discard_line=0U;
    AbortRoute(e?"console RX error/overflow":"immediate stop",e);
  }
  while (budget-- && console_rx.tail!=console_rx.head) {
    byte=console_rx.data[console_rx.tail]; __DMB(); console_rx.tail=(uint16_t)((console_rx.tail+1U)&(CONSOLE_CAP-1U));
    if (byte=='\r' || byte=='\n') {
      if (discard_line) { Print("Line too long; discarded.\r\n> "); discard_line=line_length=0U; }
      else if (line_length) { line[line_length]='\0'; ProcessLine(); line_length=0U; if (!exchange.raw) Print("> "); }
    } else if (byte=='\b' || byte==127U) { if (line_length) --line_length; }
    else if (!discard_line) { if (line_length>=LINE_CAP-1U) discard_line=1U; else line[line_length++]=(char)byte; }
    if (urgent_stop) return;
  }
  if (urgent_stop) return;
  if (motor_error && Busy() && route.phase!=ABORTING) {
    motor_error=0U; Fail("motor UART RX error/overflow");
  }
  ExchangePoll(); PrintRawReply(); RoutePoll();
}
