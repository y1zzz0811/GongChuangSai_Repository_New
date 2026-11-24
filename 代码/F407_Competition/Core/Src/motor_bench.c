#include "motor_bench.h"
#include "chassis_config.h"
#include "route1_config.h"
#include "route_library.h"
#include "qr_bridge.h"
#include <string.h>
#include "emm_bus.h"
#include "app_console.h"
#define Print App_Print
#define Log App_Log
#define Number App_ParseU32
#define snprintf App_Format
#define exchange bus.exchange
#define bus_ready bus.ready_at
#define motor_error bus.rx_error
#define ExchangeCancel() Emm_Cancel(&bus)
#define ExchangeStart(f,n,e,r) Emm_Start(&bus,f,n,e,r)
#define ExchangePoll() Emm_Poll(&bus)
#define TakeReply() Emm_TakeReply(&bus)
#define PrintRawReply() Emm_PrintRawReply(&bus)
#define SendStopBroadcast() Emm_StopBroadcast(&bus)

#define LINE_CAP 160U
#define FRAME_CAP 4U /* Exact query frames only. */
typedef enum { IDLE,PREFLIGHT_STOP,CONFIG,OPTIONS,MOTOR_ENABLE,BASELINE,LOAD,TRIGGER,
               WAIT_STATE,WAIT_POSITION,PAUSE,ABORTING,FAULT,WAIT_QR,WAIT_OPERATOR,WAIT_WORK } Phase;
typedef enum { FORWARD,BACKWARD,LEFT,RIGHT,TURN_LEFT,TURN_RIGHT } Action;
/* Mode 0=single, 1=square, 2=legacy path, 3=mission leg. */
typedef struct {
  Phase phase; Action action,requested_action;
  RouteStation from,to;
  uint8_t square,path;
  uint8_t step,count,index,arrived,stable,abort_fault,stop_failed,flags[4];
  uint32_t distance,angle,pulses,rpm,deadline,next_at;
  int64_t baseline[4],position[4];
  char reason[80];
} Route;
static UART_HandleTypeDef *motor;
static EmmBus bus;
static char line[LINE_CAP];
static uint8_t address;
static Route route;
static uint32_t length_mm,width_mm,linear_rpm,turn_rpm,straight_scale,turn_scale;
/* Finite differences of existing encoder queries; no extra bus traffic.
 * Magnitudes only: encoder polarity and mecanum ground slip are not calibrated. */
static int64_t speed_previous[4];
static uint32_t speed_tick[4],speed_rpm_milli[4],speed_sequence,speed_report;
static uint8_t speed_mask;
static void SpeedSample(uint8_t i,int64_t position,uint32_t now) {
  uint32_t dt=now-speed_tick[i]; int64_t delta=position-speed_previous[i];
  if (dt>=20U && dt<=2000U) {
    uint64_t magnitude=(uint64_t)(delta<0?-delta:delta);
    uint64_t rpm=magnitude*60000000ULL/((uint64_t)65536U*dt);
    if(rpm<=3000000ULL) { speed_rpm_milli[i]=(uint32_t)rpm; speed_mask|=(uint8_t)(1U<<i); }
    else speed_mask&=(uint8_t)~(1U<<i);
  } else speed_mask&=(uint8_t)~(1U<<i);
  speed_previous[i]=position; speed_tick[i]=now;
}
static void SpeedReport(uint32_t now) {
  if(!AppConsole_Telemetry()) return;
  uint32_t average,mm_s,mdeg_s=0U,progress=0U; unsigned i;
  uint64_t travelled=0U,expected;
  if(speed_mask!=15U || now-speed_report<200U || AppConsole_StopPending()) return;
  for(i=0U;i<4U;++i) if(now-speed_tick[i]>500U) return;
  speed_report=now;
  average=(speed_rpm_milli[0]+speed_rpm_milli[1]+speed_rpm_milli[2]+speed_rpm_milli[3])/4U;
  mm_s=(uint32_t)((uint64_t)average*CHASSIS_WHEEL_DIAMETER_MM*3141593ULL/60000000000ULL);
  if(route.action>=TURN_LEFT && length_mm+width_mm) {
    mdeg_s=(uint32_t)((uint64_t)average*CHASSIS_WHEEL_DIAMETER_MM*6U/(length_mm+width_mm));
    mm_s=0U;
  }
  for(i=0U;i<4U;++i) {
    int64_t delta=route.position[i]-route.baseline[i];
    travelled+=(uint64_t)(delta<0?-delta:delta);
  }
  expected=(uint64_t)route.pulses*65536U*4U/CHASSIS_PULSES_PER_REV;
  if(expected) progress=(uint32_t)(travelled>=expected?1000U:travelled*1000U/expected);
  Log("@motion {\"seq\":%lu,\"source\":\"encoder\",\"kind\":\"%s\",\"wheel_rpm_milli\":[%lu,%lu,%lu,%lu],\"speed_mm_s\":%lu,\"turn_mdeg_s\":%lu,\"progress_permille\":%u}\r\n",
    (unsigned long)++speed_sequence,route.action>=TURN_LEFT?"rotation":"translation",
    (unsigned long)speed_rpm_milli[0],(unsigned long)speed_rpm_milli[1],(unsigned long)speed_rpm_milli[2],(unsigned long)speed_rpm_milli[3],
    (unsigned long)mm_s,(unsigned long)mdeg_s,(unsigned)progress);
}
static const uint8_t direction_base[4]=CHASSIS_DIRECTION_BASE;
static int Due(uint32_t now,uint32_t target) { return (int32_t)(now-target)>=0; }
static uint8_t Busy(void) { return route.phase!=IDLE && route.phase!=FAULT; }
static const char *PhaseName(Phase p) {
  static const char *n[]={"idle","preflight-stop","config","options","enable","baseline",
    "preload","trigger","wait-state","wait-position","pause","stopping","fault","wait-qr","wait-operator","wait-work"}; return n[(unsigned)p];
}
static const char *ActionName(Action a) {
  static const char *n[]={"forward","backward","left","right","turn-left","turn-right"}; return n[(unsigned)a];
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
  Log("FAULT %s M%u %02X: %s\r\n",PhaseName(route.phase),
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
  /* User-confirmed C5.2 translation frame, expressed in C5.1 wiring actions:
   * new F=old L, new B=old R, new L=old F, new R=old B.
   * User confirmed the existing yaw directions, so their frames are preserved. */
  static const uint8_t legacy_action[]={LEFT,RIGHT,FORWARD,BACKWARD,TURN_LEFT,TURN_RIGHT};
  uint8_t addr=(uint8_t)(i+1U),rev=0U;
  switch (legacy_action[route.action]) {
    case FORWARD: rev=CHASSIS_FORE_AFT_REVERSED; break;
    case BACKWARD: rev=1U^CHASSIS_FORE_AFT_REVERSED; break;
    case LEFT: rev=addr==CHASSIS_FRONT_LEFT_ADDR || addr==CHASSIS_REAR_RIGHT_ADDR; break;
    case RIGHT: rev=addr==CHASSIS_FRONT_RIGHT_ADDR || addr==CHASSIS_REAR_LEFT_ADDR; break;
    case TURN_RIGHT: rev=addr==CHASSIS_FRONT_RIGHT_ADDR || addr==CHASSIS_REAR_RIGHT_ADDR; break;
    case TURN_LEFT: rev=addr==CHASSIS_FRONT_LEFT_ADDR || addr==CHASSIS_REAR_LEFT_ADDR; break;
  }
  return direction_base[i]^rev;
}
static void BeginStep(void) {
  uint64_t nominal; RouteStep entry={0};
  if (route.square==2U) {
    unsigned a;
    if(!RouteLibrary_Step(route.path,route.step,&entry)) { Fail("invalid route step"); return; }
    a=entry.action;
    /* Open this run's scan window BEFORE the QR approach. Keep any complete,
     * queued or partial frame through arrival and encoder/settle confirmation. */
    if (entry.flags & R_CLEAR_QR) QrBridge_Clear();
    if (a>=R_SCAN) {
      if (a==R_SCAN) {
        route.phase=WAIT_QR; route.deadline=HAL_GetTick()+ROUTE1_QR_TIMEOUT_MS;
        Log("ROUTE%u: check QR.\r\n",route.path);
      } else {
        static const char *const names[]={"raw","rough","temp"};
        route.phase=entry.group && a!=R_RAW?WAIT_WORK:WAIT_OPERATOR;
        AppConsole_ClearInput(); /* queued pre-arrival 'next' is not consent */
        Log("ROUTE%u %s: %s\r\n",route.path,names[a-R_RAW],
            a==R_RAW && entry.group?"place material; route next when clear.":"station.");
      }
      return;
    }
    route.action=(Action)a;
    if (a>=TURN_LEFT) route.angle=entry.value;
    else route.distance=entry.value;
  } else if (route.square==3U) {
    if (!RouteLibrary_LegStep(route.from,route.to,route.step,&entry)) { Fail("invalid mission leg"); return; }
    if (entry.flags & R_CLEAR_QR) QrBridge_Clear();
    route.action=(Action)entry.action;
    if (route.action>=TURN_LEFT) route.angle=entry.value;
    else route.distance=entry.value;
  } else route.action=route.square?((route.step&1U)?TURN_RIGHT:FORWARD):route.requested_action;
  if (!route.rpm || route.square)
    route.rpm=entry.rpm?entry.rpm:(route.action>=TURN_LEFT?turn_rpm:linear_rpm);
  route.pulses=route.action>=TURN_LEFT?TurnPulses(route.angle):DistancePulses(route.distance);
  nominal=(uint64_t)route.pulses*60000U/(route.rpm*CHASSIS_PULSES_PER_REV);
  route.deadline=HAL_GetTick()+(uint32_t)(nominal*3U+10000U);
  route.phase=BASELINE; route.index=route.arrived=route.stable=0U;
  speed_mask=0U;
  Log("STEP %u/%u: %s %lu %s pulses=%lu rpm=%lu\r\n",(unsigned)(route.step+1U),
    (unsigned)route.count,ActionName(route.action),(unsigned long)(route.action>=TURN_LEFT?route.angle:route.distance),
    route.action>=TURN_LEFT?"deg":"mm",(unsigned long)route.pulses,(unsigned long)route.rpm);
}
static void ContinuePlan(void) {
  if (++route.step>=route.count) { route.phase=IDLE; Print("DONE: verify ground pose.\r\n> "); }
  else BeginStep();
}
static void ResumeRoute1(void) {
  if(route.step+1U>=route.count) { route.step=route.count; route.phase=IDLE; Print("DONE: verify ground pose.\r\n> "); return; }
  ++route.step; route.index=0U; route.phase=PREFLIGHT_STOP;
  route.deadline=HAL_GetTick()+15000U; /* recheck all drivers after a wait */
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
  if (route.phase==WAIT_OPERATOR || route.phase==WAIT_WORK) return; /* work has its own timeout */
  if (route.phase!=ABORTING && Due(now,route.deadline)) { Fail("step timeout"); return; }
  if (route.phase==WAIT_QR) {
    char task[16];
    if (QrBridge_GetCode(task,NULL,NULL)) { Log("ROUTE%u QR OK.\r\n",route.path); ResumeRoute1(); }
    return;
  }
  if (route.phase==PAUSE) {
    if (!Due(now,route.next_at)) return;
    ContinuePlan();
    return;
  }
  if (exchange.done) {
    result=TakeReply();
    if (route.phase==ABORTING) {
      if (result<0) route.stop_failed=1U;
      if (++route.index==4U) {
        route.phase=(route.abort_fault || route.stop_failed)?FAULT:IDLE;
        Print(route.stop_failed?"STOP ACK missing; inspect motors.\r\n> ":"STOP ACK M1..4.\r\n> ");
      } return;
    }
    if (result<0) { Fail("missing/invalid reply or rejected command"); return; }
    switch (route.phase) {
      case PREFLIGHT_STOP: if (++route.index==4U) { route.index=0U; route.phase=CONFIG; } break;
      case CONFIG:
        if (exchange.reply[2]!=0x21U || exchange.reply[3]!=0x15U || exchange.reply[4]!=0x19U ||
            exchange.reply[6]!=2U || exchange.reply[9]!=(uint8_t)CHASSIS_MICROSTEPS || exchange.reply[21]!=0U ||
            (exchange.reply[22]!=1U && exchange.reply[22]!=3U)) { Fail("Emm/1.8deg/microsteps/UART/6B/reply mismatch"); return; }
        /* Read-only diagnostic; never silently rewrite the driver's settings. */
        if (exchange.reply[10]!=1U) Log("M%u interpolation not ON\r\n",(unsigned)(route.index+1U));
        if (++route.index==4U) { route.index=0U; route.phase=OPTIONS; } break;
      case OPTIONS: {
        uint16_t bits=((uint16_t)exchange.reply[2]<<8)|exchange.reply[3];
        if ((bits&0x87U)!=6U) { Fail("Emm/1.8deg/RPM-scaling mismatch"); return; }
        if (++route.index==4U) { route.index=0U; route.phase=MOTOR_ENABLE; } break;
      }
      case MOTOR_ENABLE: if (++route.index==4U) BeginStep(); break;
      case BASELINE:
        if (exchange.reply[2]>1U) { Fail("invalid position sign"); return; }
        route.baseline[route.index]=DecodePosition();
        speed_previous[route.index]=route.baseline[route.index]; speed_tick[route.index]=now;
        if (++route.index==4U) { route.index=0U; route.phase=LOAD; } break;
      case LOAD: if (++route.index==4U) { route.index=0U; route.phase=TRIGGER; } break;
      case WAIT_STATE:
        route.flags[route.index]=exchange.reply[2];
        if (!(exchange.reply[2]&1U) || (exchange.reply[2]&0x0CU)) { Fail("motor disabled or stall flag"); return; }
        route.phase=WAIT_POSITION; break;
      case WAIT_POSITION:
        if (exchange.reply[2]>1U) { Fail("invalid position sign"); return; }
        route.position[route.index]=DecodePosition();
        SpeedSample(route.index,route.position[route.index],now);
        if ((route.flags[route.index]&2U) && DisplacementMatches(route.index)) route.arrived|=(uint8_t)(1U<<route.index);
        if (++route.index==4U) {
          SpeedReport(now);
          route.index=0U; if (route.arrived==15U) ++route.stable; else route.stable=0U; route.arrived=0U;
          if (route.stable>=2U) { route.phase=PAUSE; route.next_at=now+CHASSIS_SETTLE_MS;
            Print("M1..4 encoders verified.\r\n");
            /* Magnitudes are already bounded by DisplacementMatches here.
             * Signs are diagnostic: driver encoder polarity is not calibrated.
             * Wheel travel does not measure vehicle heading or ground slip. */
            for (uint8_t i=0U;i<4U;++i) Log("M%u delta=%ld counts\r\n",(unsigned)(i+1U),
                (long)(route.position[i]-route.baseline[i]));
            return; }
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
      f[1]=0xFDU; f[2]=Direction(route.index); f[3]=(uint8_t)(route.rpm>>8); f[4]=(uint8_t)route.rpm; f[5]=CHASSIS_ACCELERATION;
      f[6]=(uint8_t)(route.pulses>>24); f[7]=(uint8_t)(route.pulses>>16); f[8]=(uint8_t)(route.pulses>>8); f[9]=(uint8_t)route.pulses;
      f[10]=2U; f[11]=1U; f[12]=0x6BU; ExchangeStart(f,13U,4U,0U); break;
    case TRIGGER:
      f[0]=0U; f[1]=0xFFU; f[2]=0x66U; f[3]=0x6BU;
      if (HAL_UART_Transmit(motor,f,4U,10U)!=HAL_OK) { Fail("trigger transmit failed"); return; }
      Log("CHASSIS RUN %u/%u\r\n",(unsigned)(route.step+1U),(unsigned)route.count);
      route.phase=WAIT_STATE; route.next_at=now+100U; bus_ready=now+3U; break;
    case WAIT_STATE: f[1]=0x3AU; ExchangeStart(f,3U,4U,0U); break;
    default: break;
  }
}
static void ShowStatus(void) {
  unsigned step=route.count?(route.step<route.count?route.step+1U:route.count):0U;
  Log("state=%s step=%u/%u action=%s target=%lu pulses\r\n",PhaseName(route.phase),step,
      (unsigned)route.count,ActionName(route.action),(unsigned long)route.pulses);
  if(route.square==2U) Log("path=%u work=%u\r\n",route.path,MotorBench_WorkGroup());
  Log("geometry L=%lu W=%lu mm; wheel=%umm/%u pulses; linear-rpm=%lu turn-rpm=%lu; cal=%lu/%lu permille\r\n",
      (unsigned long)length_mm,(unsigned long)width_mm,(unsigned)CHASSIS_WHEEL_DIAMETER_MM,
      (unsigned)CHASSIS_PULSES_PER_REV,(unsigned long)linear_rpm,(unsigned long)turn_rpm,
      (unsigned long)straight_scale,(unsigned long)turn_scale);
  Log("wheel addresses FL=%u FR=%u RL=%u RR=%u\r\n",(unsigned)CHASSIS_FRONT_LEFT_ADDR,
      (unsigned)CHASSIS_FRONT_RIGHT_ADDR,(unsigned)CHASSIS_REAR_LEFT_ADDR,(unsigned)CHASSIS_REAR_RIGHT_ADDR);
  if (route.rpm) Log("last/current step rpm=%lu\r\n",(unsigned long)route.rpm);
  Log("flags M1=%02X M2=%02X M3=%02X M4=%02X; id=%u\r\n",
      route.flags[0],route.flags[1],route.flags[2],route.flags[3],address);
  Log("acc=%u (Emm level)\r\n",(unsigned)CHASSIS_ACCELERATION);
  if (route.reason[0]) Log("last stop/fault: %s\r\n",route.reason);
}
static void StartRoute(Action a,uint32_t value,uint8_t square,uint8_t path) {
  if (route.phase==FAULT) { Print("Fault latched; inspect then clearfault.\r\n"); return; }
  if (Busy() || exchange.busy || exchange.done) { Print("Busy. Use status or stopall.\r\n"); return; }
  if ((square || a>=TURN_LEFT) && (!length_mm || !width_mm)) { Print("Set measured geometry L W.\r\n"); return; }
  memset(&route,0,sizeof(route)); route.phase=PREFLIGHT_STOP; route.action=a; route.requested_action=a; route.square=square; route.path=path;
  route.count=square==2U?RouteLibrary_Count(path):(square?8U:1U);
  route.distance=a<TURN_LEFT?value:0U; route.angle=square?90U:value;
  route.deadline=HAL_GetTick()+15000U;
  if (path) Log("ROUTE%u START.\r\n",(unsigned)path);
  Print("START: preflight; ! stops.\r\n");
}
static void RawTransmit(char *args) {
  uint8_t f[FRAME_CAP],n=0U; char *t=strtok(args," \t");
  while (t) {
    if (n==FRAME_CAP || !App_ParseHexByte(t,&f[n])) {
      Print("TX query only; use motion commands or stopall.\r\n"); return;
    }
    ++n; t=strtok(NULL," \t");
  }
  /* This terminal is query-only. Arbitrary motion/config writes would bypass
   * speed limits and leave physical motion outside the vehicle busy state. */
  if (!n || !f[0] || !((n==3U && f[2]==0x6BU &&
      (f[1]==0x1AU || f[1]==0x1FU || f[1]==0x24U || f[1]==0x27U ||
       f[1]==0x36U || f[1]==0x37U || f[1]==0x3AU)) ||
      (n==4U && f[1]==0x42U && f[2]==0x6CU && f[3]==0x6BU))) {
    Print("TX query only; use motion commands or stopall.\r\n"); return;
  }
  ExchangeStart(f,n,0U,1U);
}
static void ProcessLine(void) {
  char *cmd,*a,*b,*extra; uint32_t v,w; uint8_t f[5];
  if (!strcmp(line,"stopall")) { AbortRoute("operator stopall",0U); return; }
  if (!strcmp(line,"stop") && Busy()) { AbortRoute("operator stop cancelled route",0U); return; }
  if (!strcmp(line,"status")) { ShowStatus(); return; }
  if (!strcmp(line,"next")) {
    if (route.square==2U && route.phase==WAIT_OPERATOR) {
      RouteStep e;
      if(!RouteLibrary_Step(route.path,route.step,&e)) Fail("invalid station");
      else if(e.group) route.phase=WAIT_WORK;
      else ResumeRoute1();
    }
    else Print("Route is not waiting for next.\r\n");
    return;
  }
  if (!strcmp(line,"help")) {
    Print("forward/backward/left/right MM | turn left/right DEG\r\nsquare/route1/next | speed/cal/geometry/id/readpos/tx\r\nstatus/stopall/!/clearfault\r\n"); return;
  }
  if (Busy() || exchange.busy || exchange.done) { Print("Busy: status/stopall.\r\n"); return; }
  if (!strcmp(line,"route1")) { MotorBench_StartPath(1U); return; }
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
      length_mm=v; width_mm=w; Log("RAM geometry: 90deg=%lu pulses\r\n",(unsigned long)TurnPulses(90U));
    } else Print("geometry L W mm (10..3000)\r\n");
  } else if (!strcmp(cmd,"speed")) {
    if (!b && Number(a,1U,CHASSIS_MAX_RPM,&v)) {
      linear_rpm=turn_rpm=v; /* Preserve the legacy global-speed command. */
    } else if (!extra && a && b && Number(b,1U,CHASSIS_MAX_RPM,&v) &&
               (!strcmp(a,"linear") || !strcmp(a,"turn"))) {
      if (!strcmp(a,"linear")) linear_rpm=v; else turn_rpm=v;
    } else {
      Log("Use: speed [linear|turn] RPM (1..%u)\r\n",(unsigned)CHASSIS_MAX_RPM); return;
    }
     Log("RAM RPM linear=%lu turn=%lu\r\n",(unsigned long)linear_rpm,(unsigned long)turn_rpm);
  }
  else if (!strcmp(cmd,"cal") && a && !extra && Number(b,100U,5000U,&v)) {
    if (!strcmp(a,"straight")) straight_scale=v; else if (!strcmp(a,"turn")) turn_scale=v;
    else { Print("cal straight|turn 100..5000; 1000=1.0\r\n"); return; } Print("RAM calibration set.\r\n");
  } else if (!strcmp(cmd,"turn") && a && !extra && Number(b,1U,360U,&v) && (!strcmp(a,"left") || !strcmp(a,"right"))) {
    StartRoute(!strcmp(a,"left")?TURN_LEFT:TURN_RIGHT,v,0U,0U);
  }
  else if (!strcmp(cmd,"square") && !b && Number(a,10U,3000U,&v)) StartRoute(FORWARD,v,1U,0U);
  else if (!b && Number(a,10U,3000U,&v) && (!strcmp(cmd,"forward") || !strcmp(cmd,"backward") || !strcmp(cmd,"left") || !strcmp(cmd,"right"))) {
    Action action=!strcmp(cmd,"forward")?FORWARD:!strcmp(cmd,"backward")?BACKWARD:!strcmp(cmd,"left")?LEFT:RIGHT;
    StartRoute(action,v,0U,0U);
  } else if (!strcmp(cmd,"clearfault") && !a) {
    if (route.phase==FAULT) { if (route.stop_failed) { Print("Retry stopall before clearfault.\r\n"); return; } route.phase=IDLE; Print("Fault cleared; motors recheck next move.\r\n"); }
    else Print("No latched fault.\r\n");
  } else Print("Invalid command/args; see manual.\r\n");
}

void MotorBench_Init(UART_HandleTypeDef *c,UART_HandleTypeDef *m) {
  speed_sequence=speed_report=0U; speed_mask=0U;
  memset(speed_previous,0,sizeof(speed_previous)); memset(speed_tick,0,sizeof(speed_tick)); memset(speed_rpm_milli,0,sizeof(speed_rpm_milli));
  (void)c; motor=m; Emm_Init(&bus,m,CHASSIS_REPLY_TIMEOUT_MS);
  memset(&route,0,sizeof(route)); address=1U; RouteLibrary_Init();
  length_mm=CHASSIS_LENGTH_MM; width_mm=CHASSIS_WIDTH_MM;
  linear_rpm=CHASSIS_DEFAULT_LINEAR_RPM; turn_rpm=CHASSIS_DEFAULT_TURN_RPM;
  straight_scale=CHASSIS_STRAIGHT_SCALE; turn_scale=CHASSIS_TURN_SCALE;
}
void MotorBench_Poll(void) {
  if (motor_error && Busy() && route.phase!=ABORTING) { motor_error=0U; Fail("motor UART RX error/overflow"); }
  ExchangePoll(); PrintRawReply(); RoutePoll(); }
void MotorBench_Command(const char *command) {
  if (!command || strlen(command)>=sizeof(line)) { Print("Command too long.\r\n"); return; }
  memcpy(line,command,strlen(command)+1U); ProcessLine();
}
uint8_t MotorBench_IsBusy(void) { return (uint8_t)(Busy() || exchange.busy || exchange.done || !Due(HAL_GetTick(),bus_ready)); }
uint8_t MotorBench_HasFault(void) { return (uint8_t)(route.phase==FAULT || (route.phase==ABORTING && route.abort_fault)); }
const char *MotorBench_LivePhase(uint8_t *path,uint8_t *step,uint8_t *count) {
  *path=route.path;
  *count=route.count;
  *step=route.step<route.count?route.step+1U:route.count;
  if (route.phase==WAIT_STATE || route.phase==WAIT_POSITION) return "moving";
  if (route.phase==IDLE && route.count && route.step==route.count) return "done";
  return PhaseName(route.phase);
}
void MotorBench_Stop(const char *reason,uint8_t fault) { AbortRoute(reason,fault); if (route.phase==IDLE) route.phase=ABORTING; }
void MotorBench_OnRx(UART_HandleTypeDef *u) { if (u==motor) Emm_OnRx(&bus); }
void MotorBench_OnError(UART_HandleTypeDef *u) { if (u==motor) Emm_OnError(&bus); }

uint8_t MotorBench_WorkGroup(void) {
  RouteStep e;
  return route.square==2U && route.phase==WAIT_WORK && RouteLibrary_Step(route.path,route.step,&e)?e.group:0U;
}
void MotorBench_CompleteWork(void) { if (route.phase==WAIT_WORK) ResumeRoute1(); }
uint8_t MotorBench_StartPath(uint8_t id) {
  if(MotorBench_IsBusy() || MotorBench_HasFault() || !RouteLibrary_Validate(id)) return 0U;
  StartRoute(FORWARD,0U,2U,id); return route.phase==PREFLIGHT_STOP;
}
uint8_t MotorBench_StartLeg(RouteStation from,RouteStation to) {
  uint8_t count=RouteLibrary_LegCount(from,to);
  if (!count || MotorBench_IsBusy() || MotorBench_HasFault()) return 0U;
  StartRoute(FORWARD,0U,3U,2U);
  if (route.phase!=PREFLIGHT_STOP) return 0U;
  route.from=from; route.to=to; route.count=count;
  return 1U;
}
static uint8_t StartTranslation(int16_t signed_mm,Action positive,Action negative,
                                uint32_t minimum,uint32_t maximum) {
  uint32_t distance=(uint32_t)(signed_mm<0?-(int32_t)signed_mm:signed_mm);
  if (distance<minimum || distance>maximum || MotorBench_IsBusy() || MotorBench_HasFault()) return 0U;
  StartRoute(signed_mm>0?positive:negative,distance,0U,0U);
  return route.phase==PREFLIGHT_STOP;
}

uint8_t MotorBench_Shift(int16_t right_mm) { return StartTranslation(right_mm,RIGHT,LEFT,10U,300U); }

uint8_t MotorBench_TrimShift(int16_t right_mm) {
  if (!StartTranslation(right_mm,RIGHT,LEFT,1U,100U)) return 0U;
  route.rpm=10U;
  return 1U;
}

uint8_t MotorBench_TrimTravel(int16_t forward_mm) {
  if (!StartTranslation(forward_mm,FORWARD,BACKWARD,1U,30U)) return 0U;
  route.rpm=10U;
  return 1U;
}
