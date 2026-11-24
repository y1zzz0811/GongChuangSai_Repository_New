/* Derived from F407 MotorBench's buffered UART/Emm transaction engine.
 * Only USART6 M5..M7. No chassis, servo, homing or automatic startup moves. */
#include "arm_bench.h"
#include "arm_config.h"
#include "arm_units.h"
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
#define PICKUP_RAIL_ACTION 3U
typedef enum { IDLE,PREFLIGHT_STOP,CONFIG,OPTIONS,MOTOR_ENABLE,BASELINE,LOAD,
               TRIGGER,WAIT_STATE,WAIT_POSITION,ABORTING,FAULT,CHECK_LIFT } Phase;
typedef struct { uint32_t pulses,rpm; uint8_t direction; } Move;
typedef struct {
  int64_t low,high;
  uint32_t since,last;
  uint8_t samples;
} PositionWindow;
typedef struct {
  Phase phase;
  uint8_t mask,index,arrived,stable,flags[3],abort_fault,stop_failed,sync,sampled;
  uint8_t route_action,returning;
  uint64_t rail_from,rail_to;
  int64_t pickup_target;
  int16_t pickup_offset_mm;
  Move move[3];
  int64_t baseline[3],position[3];
  uint32_t deadline,next_at,check_until;
  PositionWindow window;
  const char *waiting;
  char reason[80];
} Motion;
static const uint32_t full_steps[3]={ARM_M5_FULL_STEPS,ARM_M6_FULL_STEPS,ARM_M7_FULL_STEPS};
static const uint32_t microsteps[3]={ARM_M5_MICROSTEPS,ARM_M6_MICROSTEPS,ARM_M7_MICROSTEPS};
static const uint32_t max_travel[3]={ARM_M5_MAX_TRAVEL_MILLI,ARM_M6_MAX_TRAVEL_MILLI,ARM_M7_MAX_TRAVEL_MILLI};
static const uint32_t max_rpm[3]={ARM_M5_MAX_RPM,ARM_M6_MAX_RPM,ARM_M7_MAX_RPM};
static const char *axis_names[3]={"base","lift","rail"};
static UART_HandleTypeDef *motor;
static EmmBus bus;
static char line[LINE_CAP];
static uint8_t armed,plan_mask;
static Motion motion;
static int64_t pickup_origin;
static int8_t pickup_forward_sign;
static Move plan[3];
static uint32_t default_rpm[3];
static uint32_t fixed_travel[3];
static int64_t speed_position[3];
static uint32_t speed_tick[3],speed_milli[3],progress_permille[3],speed_report;
static uint8_t speed_mask;
typedef enum { REF_IDLE,REF_CAPTURE,REF_START } RefMode;
typedef enum { REF_UNCHECKED,REF_CHECKING,REF_VERIFIED,REF_MISMATCH } RefCheck;
static struct {
  RefMode mode;
  RefCheck check;
  uint8_t axis,valid;
  int64_t zero[3];
  int8_t lift_down_sign;
  uint32_t check_until,next_at;
  PositionWindow window;
} reference;
static uint32_t lift_report;

typedef enum { ADDR_IDLE,ADDR_DISCOVER,ADDR_STOP,ADDR_WRITE,ADDR_VERIFY } AddressPhase;
static struct {
  AddressPhase phase;
  uint8_t changing,old_id,new_id;
} address_op;
static int Due(uint32_t now,uint32_t target) { return (int32_t)(now-target)>=0; }
static uint8_t Busy(void) { return motion.phase!=IDLE && motion.phase!=FAULT; }
static const char *PhaseName(Phase p) {
  static const char *names[]={"idle","preflight-stop","config","options","enable",
    "baseline","preload","trigger","wait-state","wait-position","stopping","fault","check-lift"};
  return names[(unsigned)p];
}


static void ClearPlan(void) { memset(plan,0,sizeof(plan)); plan_mask=0U; }
static void AbortMotion(const char *reason,uint8_t fault) {
  uint8_t was_fault=(uint8_t)(motion.phase==FAULT || (motion.phase==ABORTING && motion.abort_fault));
  address_op.phase=ADDR_IDLE;
  reference.mode=REF_IDLE; reference.valid=0U; reference.check=REF_UNCHECKED;
  ExchangeCancel(); SendStopBroadcast(); ClearPlan(); armed=0U;
  motion.abort_fault=(uint8_t)(fault || was_fault);
  motion.stop_failed=0U; motion.index=0U; motion.mask=7U; motion.phase=ABORTING;
  if (!was_fault) (void)snprintf(motion.reason,sizeof(motion.reason),"%s",reason);
  Print("STOP USART6 M5..7; arm locked.\r\n");
}
static void Fail(const char *reason) {
  Log("FAULT at %s M%u: %s\r\n",PhaseName(motion.phase),(unsigned)(motion.index+5U),reason);
  AbortMotion(reason,1U);
}
static int64_t DecodePosition(void) {
  uint32_t value=((uint32_t)exchange.reply[3]<<24)|((uint32_t)exchange.reply[4]<<16)|
                 ((uint32_t)exchange.reply[5]<<8)|exchange.reply[6];
  return exchange.reply[2]?-(int64_t)value:(int64_t)value;
}
static uint64_t Magnitude(int64_t value) { return (uint64_t)(value<0?-value:value); }
static uint64_t EncoderTravel(uint8_t i,uint32_t pulses) {
  uint32_t ppr=full_steps[i]*microsteps[i];
  return ((uint64_t)pulses*65536U+ppr/2U)/ppr;
}
/* Convert a physical tolerance directly to encoder counts, rounding down. */
static uint64_t RouteCounts(uint8_t i,uint32_t um) {
  const ArmUnitProfile *p=ArmUnits_Get(i);
  return (uint64_t)um*p->pulses*65536U/
      ((uint64_t)p->travel_milli*full_steps[i]*microsteps[i]);
}
static uint64_t RouteTolerance(uint8_t i,uint64_t target) {
  if (i==2U) return RouteCounts(i,ARM_M7_ROUTE_TOLERANCE_UM);
  if (i==1U && target) return RouteCounts(i,ARM_M6_ROUTE_TOLERANCE_UM);
  return ARM_POSITION_TOLERANCE; /* M5 angle and M6 top clearance */
}
static uint8_t NearReference(uint8_t i,int64_t position,uint64_t target) {
  uint64_t actual=Magnitude(position-reference.zero[i]);
  uint64_t error=actual>target?actual-target:target-actual;
  return (uint8_t)(reference.valid && error<=RouteTolerance(i,target));
}
static uint8_t StablePosition(PositionWindow *window,uint8_t axis,int64_t position,
                              uint8_t in_range,uint32_t now) {
  if (!in_range) { window->samples=0U; return 0U; }
  if (!window->samples || now-window->last>120U ||
      Magnitude(position-window->low)>RouteCounts(axis,ARM_ROUTE_STABLE_SPAN_UM) ||
      Magnitude(position-window->high)>RouteCounts(axis,ARM_ROUTE_STABLE_SPAN_UM)) {
    window->low=window->high=position;
    window->since=now;
    window->samples=0U;
  }
  if (position<window->low) window->low=position;
  if (position>window->high) window->high=position;
  window->last=now;
  if (window->samples<5U) ++window->samples;
  return window->samples==5U && now-window->since>=ARM_ROUTE_STABLE_MS;
}
static void ReportWait(uint8_t axis,const char *reason) {
  if (motion.waiting!=reason) {
    motion.waiting=reason;
    Log("@armwait %u %s\r\n",(unsigned)(axis+5U),reason?reason:"none");
  }
}
static void ShowReference(void) {
  if (!reference.valid) { Print("ARM REF unset; place M5/M6/M7 at zero and use arm ref set.\r\n"); return; }
  for (unsigned i=0U;i<3U;++i) {
    int64_t zero=reference.zero[i];
    Log("ARM REF M%u zero=%c%lu counts\r\n",i+5U,zero<0?'-':'+',(unsigned long)Magnitude(zero));
  }
}
static void ReferencePoll(void) {
  uint8_t i=reference.axis;
  if (exchange.done) {
    int result=TakeReply();
    if (result<0 || exchange.reply[2]>1U) {
      Log("ARM REF failed: M%u encoder reply.\r\n",(unsigned)(i+5U));
      reference.valid=0U; reference.mode=REF_IDLE; reference.check=REF_MISMATCH; return;
    }
    int64_t position=DecodePosition();
    if (reference.mode==REF_CAPTURE) reference.zero[i]=position;
    else {
      uint64_t travel=Magnitude(position-reference.zero[i]);
      uint8_t within=NearReference(i,position,0U);
      if (i && !StablePosition(&reference.window,i,position,within,HAL_GetTick())) {
        if (!Due(HAL_GetTick(),reference.check_until)) {
          reference.next_at=HAL_GetTick()+25U;
          return;
        }
        within=0U;
      }
      if (!within) {
        Log("START BLOCKED: M%u offset=%lu counts, expected=%lu.\r\n",
          (unsigned)(i+5U),(unsigned long)(travel>UINT32_MAX?UINT32_MAX:travel),0UL);
        reference.mode=REF_IDLE; reference.check=REF_MISMATCH; return;
      }
    }
    reference.window.samples=0U;
    reference.check_until=HAL_GetTick()+ARM_ROUTE_RECHECK_MS;
    if (++reference.axis==(reference.mode==REF_CAPTURE?3U:2U)) {
      if (reference.mode==REF_CAPTURE) { reference.valid=1U; ShowReference(); }
      else { reference.check=REF_VERIFIED; Print("ARM REF pose verified: M5=0, M6=0.\r\n"); }
      reference.mode=REF_IDLE;
    }
  } else if (!exchange.busy && Due(HAL_GetTick(),bus_ready) &&
             (reference.mode!=REF_START || Due(HAL_GetTick(),reference.next_at)) && !AppConsole_StopPending()) {
    uint8_t f[]={(uint8_t)(i+5U),0x36U,0x6BU};
    ExchangeStart(f,sizeof(f),8U,0U);
  }
}
void ArmBench_ReportReadiness(void) {
  unsigned zero=reference.mode==REF_CAPTURE?2U:reference.valid;
  App_Log("@prep %u %u\r\n",zero,(unsigned)reference.check);
}
/* Report from existing position reads. flags=-1 denotes the pre-turn check,
 * which reads position only. verified is the result of the applicable check. */
static void ReportLift(int64_t position,int flags,uint8_t verified) {
  if(!AppConsole_Telemetry()) return;
  uint64_t counts=Magnitude(position-reference.zero[1]);
  uint64_t pulses=(counts*full_steps[1]*microsteps[1]+32768U)/65536U;
  uint32_t height;
  if (!reference.valid || pulses>UINT32_MAX || !ArmUnits_FromPulses(1U,(uint32_t)pulses,&height) ||
      AppConsole_StopPending()) return;
  App_Log("@lift %lu %d %u\r\n",(unsigned long)height,flags,(unsigned)verified);
}
static void SampleAxis(uint8_t i,uint32_t now) {
  uint32_t dt=now-speed_tick[i];
  uint8_t bit=(uint8_t)(1U<<i);
  int64_t delta=motion.position[i]-speed_position[i];
  int64_t travel=motion.position[i]-motion.baseline[i];
  uint64_t actual=(uint64_t)(travel<0?-travel:travel);
  uint64_t expected=(uint64_t)motion.move[i].pulses*65536U/(full_steps[i]*microsteps[i]);
  speed_mask&=(uint8_t)~bit;
  if ((motion.sampled&bit) && dt>=20U && dt<=2000U) {
    uint64_t rpm=(uint64_t)(delta<0?-delta:delta)*60000000ULL/(65536ULL*dt);
    if (rpm<=3000000U) { speed_milli[i]=(uint32_t)rpm; speed_mask|=bit; }
  }
  progress_permille[i]=!expected?0U:actual>=expected?1000U:(uint32_t)(actual*1000U/expected);
  speed_tick[i]=now;
  speed_position[i]=motion.position[i];
}
static void ReportAxes(uint32_t now) {
  if(!AppConsole_Telemetry()) return;
  if (!speed_mask || now-speed_report<200U || AppConsole_StopPending()) return;
  speed_report=now;
  Log("@axes %u %lu %lu %lu %lu %lu %lu\r\n",(unsigned)speed_mask,
    (unsigned long)speed_milli[0],(unsigned long)speed_milli[1],(unsigned long)speed_milli[2],
    (unsigned long)progress_permille[0],(unsigned long)progress_permille[1],(unsigned long)progress_permille[2]);
}
static void NextAxis(Phase next) {
  do { ++motion.index; } while (motion.index<3U && !(motion.mask&(1U<<motion.index)));
  if (motion.index==3U) {
    motion.index=0U;
    while (!(motion.mask&(1U<<motion.index))) ++motion.index;
    motion.phase=next;
  }
}
static uint8_t PositionMatches(uint8_t i) {
  int64_t delta=motion.position[i]-motion.baseline[i];
  uint64_t target=EncoderTravel(i,motion.move[i].pulses);
  uint64_t actual=(uint64_t)(delta<0?-delta:delta),tol=target/4U;
  /* Driver positive-direction settings can invert encoder sign. Verify travel magnitude. */
  if (tol>ARM_POSITION_TOLERANCE) tol=ARM_POSITION_TOLERANCE;
  if (tol<2U) tol=2U;
  if (motion.route_action==PICKUP_RAIL_ACTION) {
    return (uint8_t)(actual+tol>=target && actual<=target+tol &&
      (!pickup_forward_sign ||
       Magnitude(motion.position[i]-motion.pickup_target)<=RouteTolerance(i,0U)));
  }
  if (motion.route_action && i) {
    uint64_t from=motion.route_action==2U?motion.rail_from:motion.returning?target:0U;
    uint64_t to=motion.route_action==2U?motion.rail_to:motion.returning?0U:target;
    /* A good absolute endpoint can have a larger relative residual if the
     * starting point was itself within tolerance. Keep the small-move cap. */
    uint64_t budget=RouteTolerance(i,from)+RouteTolerance(i,to);
    if (!NearReference(i,motion.position[i],to)) return 0U;
    if (motion.route_action==2U &&
        ((motion.position[i]>reference.zero[i])!=(motion.baseline[i]>reference.zero[i]))) return 0U;
    if (target/4U<budget) budget=target/4U;
    if (budget<2U) budget=2U;
    return actual+budget>=target && actual<=target+budget;
  }
  return (uint8_t)(actual+tol>=target && actual<=target+tol &&
                   (!motion.route_action || NearReference(i,motion.position[i],motion.returning?0U:target)));
}
static uint32_t PulseLimit(uint8_t i);

/* Returns 0 for an invalid target, 1 for a move, 2 if already at target. */
static uint8_t PreparePickupRail(void) {
  const uint32_t ppr=full_steps[2]*microsteps[2];
  uint32_t total_pulses;
  int64_t travel,remaining;
  int16_t offset=motion.pickup_offset_mm;
  if (!pickup_forward_sign) {
    pickup_origin=motion.baseline[2];
    Log("PICKUP TEST M7 return origin=%c%lu counts\r\n",pickup_origin<0?'-':'+',
        (unsigned long)Magnitude(pickup_origin));
    return 1U;
  }
  if (!ArmUnits_ToPulses(2U,(uint32_t)(offset<0?-offset:offset)*1000U,&total_pulses)) return 0U;
  travel=(int64_t)EncoderTravel(2U,total_pulses)*(offset<0?-pickup_forward_sign:pickup_forward_sign);
  motion.pickup_target=pickup_origin+travel;
  remaining=motion.pickup_target-motion.baseline[2];
  uint64_t pulses=(Magnitude(remaining)*ppr+32768U)/65536U;
  if (!pulses) return 2U;
  if (pulses>PulseLimit(2U)) return 0U;
  motion.move[2].pulses=(uint32_t)pulses;
  motion.move[2].direction=ArmUnits_Get(2U)->positive_direction^((remaining>0)!=(pickup_forward_sign>0));
  return 1U;
}

static uint8_t SetLiftReturnDistance(int64_t from) {
  int64_t offset=from-reference.zero[1];
  int8_t side=(int8_t)((offset>0)-(offset<0));
  uint32_t ppr=full_steps[1]*microsteps[1];
  /* Round toward the working top without requesting an extra pulse past it.
   * Allow half a count for the rounding of the encoder conversion. */
  uint64_t pulses=(Magnitude(offset)*ppr+ppr/2U)/65536U;
  if (!reference.lift_down_sign || side!=reference.lift_down_sign ||
      !pulses || pulses>PulseLimit(1U)) return 0U;
  Log("M6 return target: zero=%c%lu start=%c%lu pulses=%lu (nominal=%lu)\r\n",
      reference.zero[1]<0?'-':'+',(unsigned long)Magnitude(reference.zero[1]),
      from<0?'-':'+',(unsigned long)Magnitude(from),
      (unsigned long)pulses,(unsigned long)motion.move[1].pulses);
  motion.move[1].pulses=(uint32_t)pulses;
  return 1U;
}
static void MotionPoll(void) {
  uint32_t now=HAL_GetTick(); uint8_t f[13],i=motion.index; int result;
  if (!Busy()) return;
  if (motion.phase!=ABORTING && Due(now,motion.deadline)) {
    if (motion.waiting) Log("ARM completion wait: %s\r\n",motion.waiting);
    if (motion.route_action && i==1U && (motion.sampled&2U)) {
      Log("M6 completion feedback: zero=%c%lu pos=%c%lu flags=%02X\r\n",
          reference.zero[1]<0?'-':'+',(unsigned long)Magnitude(reference.zero[1]),
          motion.position[1]<0?'-':'+',(unsigned long)Magnitude(motion.position[1]),
          (unsigned)motion.flags[1]);
    }
    Fail(motion.route_action && motion.returning && i==1U?"M6 return to recorded top not verified":"movement timeout"); return;
  }
  if (exchange.done) {
    result=TakeReply();
    if (motion.phase==ABORTING) {
      if (result<0) motion.stop_failed=1U;
      if (++motion.index==3U) {
        motion.phase=(motion.abort_fault || motion.stop_failed)?FAULT:IDLE;
        Print(motion.stop_failed?"STOP ACK missing; support lift.\r\n> ":
              "STOP ACK M5..7; verify standstill.\r\n> ");
      }
      return;
    }
    if (result<0) { Fail("missing/invalid reply or rejected command"); return; }
    switch (motion.phase) {
      case PREFLIGHT_STOP: NextAxis(CONFIG); break;
      case CONFIG:
        /* Same Emm configuration schema as the working chassis baseline.
         * Reject other schemas rather than silently guess a 35/28 driver profile. */
        if (exchange.reply[2]!=0x21U || exchange.reply[3]!=0x15U ||
            exchange.reply[4]!=(full_steps[i]==400U?0x32U:0x19U) ||
            exchange.reply[6]!=2U || exchange.reply[9]!=(uint8_t)(microsteps[i]==256U?0U:microsteps[i]) ||
            exchange.reply[21]!=0U || (exchange.reply[22]!=1U && exchange.reply[22]!=3U)) {
          Fail("driver config/UART/6B/microsteps mismatch"); return;
        }
        NextAxis(OPTIONS); break;
      case OPTIONS: {
        uint16_t bits=(uint16_t)(((uint16_t)exchange.reply[2]<<8)|exchange.reply[3]);
        uint16_t wanted=(uint16_t)(6U | (full_steps[i]==400U?1U:0U));
        if ((bits&0x87U)!=wanted) { Fail("Emm/step-angle/RPM-scaling mismatch"); return; }
        NextAxis(MOTOR_ENABLE); break;
      }
      case MOTOR_ENABLE:
        NextAxis(BASELINE);
        motion.check_until=now+ARM_ROUTE_RECHECK_MS;
        motion.next_at=now;
        break;
      case BASELINE:
        if (exchange.reply[2]>1U) { Fail("invalid position sign"); return; }
        motion.baseline[i]=DecodePosition();
        if (motion.route_action==PICKUP_RAIL_ACTION) {
          uint8_t pickup=PreparePickupRail();
          if (!pickup) { Fail("pickup M7 target invalid"); return; }
          if (pickup==2U) { motion.phase=IDLE; Print("DONE: M7 already at pickup target.\r\n> "); return; }
        } else if (motion.route_action && !NearReference(i,motion.baseline[i],
            motion.route_action==2U?motion.rail_from:
            motion.returning?EncoderTravel(i,motion.move[i].pulses):0U)) {
          if (i && !Due(now,motion.check_until)) {
            ReportWait(i,"start_reference");
            motion.next_at=now+25U;
            return;
          }
          Fail("route arm start does not match reference"); return;
        }
        if (motion.route_action && i==1U && motion.returning &&
            !SetLiftReturnDistance(motion.baseline[i])) {
          Fail("M6 return direction or travel not verified"); return;
        }
        ReportWait(i,NULL);
        motion.check_until=now+ARM_ROUTE_RECHECK_MS;
        NextAxis((motion.route_action && i==0U) || motion.route_action>=2U?CHECK_LIFT:LOAD); break;
      case CHECK_LIFT: {
        if (exchange.reply[2]>1U) { Fail("invalid M6 position sign"); return; }
        int64_t position=DecodePosition();
        uint8_t top=NearReference(1U,position,0U);
        if (!StablePosition(&motion.window,1U,position,top,now)) {
          ReportWait(1U,"top_clearance");
          if (now-lift_report>=200U) { lift_report=now; ReportLift(position,-1,0U); }
          if (Due(now,motion.check_until)) {
            Fail(motion.move[i].pulses?"M5 rotation or M7 compensation blocked: M6 not at recorded top":
                 "Tray release height check failed: M6 not at recorded top");
            return;
          }
          motion.next_at=now+25U;
          return;
        }
        ReportLift(position,-1,1U);
        motion.window.samples=0U;
        ReportWait(1U,NULL);
        if (!motion.move[i].pulses) {
          motion.phase=IDLE;
          Print("DONE: M6 verified at tray release height.\r\n> ");
          return;
        }
        motion.phase=LOAD; break;
      }
      case LOAD:
        NextAxis(motion.sync?TRIGGER:WAIT_STATE);
        if (motion.phase==WAIT_STATE) motion.next_at=now+100U;
        break;
      case WAIT_STATE:
        motion.flags[i]=exchange.reply[2];
        if (!(motion.flags[i]&1U) || (motion.flags[i]&0x0CU)) { Fail("motor disabled or stalled"); return; }
        motion.phase=WAIT_POSITION; break;
      case WAIT_POSITION:
        if (exchange.reply[2]>1U) { Fail("invalid position sign"); return; }
        motion.position[i]=DecodePosition(); SampleAxis(i,now); motion.sampled|=(uint8_t)(1U<<i);
        {
          uint8_t position_ok=PositionMatches(i);
          uint8_t driver_ready=(motion.flags[i]&2U)!=0U;
          uint8_t ready=position_ok && driver_ready;
          if (motion.route_action && i) {
            uint8_t stable=StablePosition(&motion.window,i,motion.position[i],position_ok,now);
            ready=(uint8_t)(ready && stable);
            ReportWait(i,!position_ok?"position":!stable?"stabilizing":!driver_ready?"driver_arrival":NULL);
          }
          if (ready) motion.arrived|=(uint8_t)(1U<<i);
        }
        NextAxis(WAIT_STATE);
        if (motion.index<=i) {
          ReportAxes(now);
          if (motion.arrived==motion.mask) ++motion.stable; else motion.stable=0U;
          motion.arrived=0U;
          if (motion.route_action && i==1U && (motion.stable>=2U || now-lift_report>=200U)) {
            lift_report=now;
            ReportLift(motion.position[i],motion.flags[i],(uint8_t)(motion.stable>=2U));
          }
          if (motion.stable>=2U) {
            if (motion.route_action==PICKUP_RAIL_ACTION && !pickup_forward_sign) {
              uint8_t moved_positive=motion.position[2]>motion.baseline[2];
              uint8_t commanded_forward=motion.move[2].direction==ArmUnits_Get(2U)->positive_direction;
              pickup_forward_sign=moved_positive==commanded_forward?1:-1;
            }
            if (motion.route_action && i==1U && !motion.returning) {
              int64_t offset=motion.position[i]-reference.zero[1];
              int64_t movement=motion.position[i]-motion.baseline[i];
              int8_t side=(int8_t)((offset>0)-(offset<0));
              if (!side || !movement || (offset>0)!=(movement>0) ||
                  (reference.lift_down_sign && reference.lift_down_sign!=side)) {
                Fail("M6 descent conflicts with recorded lift direction"); return;
              }
              reference.lift_down_sign=side;
            }
            motion.phase=IDLE; Print("DONE: arm encoders verified.\r\n> "); return;
          }
        }
        motion.phase=WAIT_STATE; motion.next_at=now+25U; break;
      default: break;
    }
    return;
  }
  if (exchange.busy || !Due(now,bus_ready)) return;
  if ((motion.phase==WAIT_STATE || motion.phase==WAIT_POSITION ||
       motion.phase==BASELINE || motion.phase==CHECK_LIFT) && !Due(now,motion.next_at)) return;
  f[0]=(uint8_t)(i+5U); f[2]=0x6BU;
  switch (motion.phase) {
    case PREFLIGHT_STOP: case ABORTING:
      f[1]=0xFEU; f[2]=0x98U; f[3]=0U; f[4]=0x6BU; ExchangeStart(f,5U,4U,0U); break;
    case CONFIG: f[1]=0x42U; f[2]=0x6CU; f[3]=0x6BU; ExchangeStart(f,4U,33U,0U); break;
    case OPTIONS: f[1]=0x1AU; ExchangeStart(f,3U,5U,0U); break;
    case MOTOR_ENABLE:
      f[1]=0xF3U; f[2]=0xABU; f[3]=1U; f[4]=0U; f[5]=0x6BU; ExchangeStart(f,6U,4U,0U); break;
    case CHECK_LIFT: f[0]=6U; /* fall through */
    case BASELINE: case WAIT_POSITION: f[1]=0x36U; ExchangeStart(f,3U,8U,0U); break;
    case LOAD:
      f[1]=0xFDU; f[2]=motion.move[i].direction;
      f[3]=(uint8_t)(motion.move[i].rpm>>8); f[4]=(uint8_t)motion.move[i].rpm; f[5]=ARM_ACCELERATION;
      f[6]=(uint8_t)(motion.move[i].pulses>>24); f[7]=(uint8_t)(motion.move[i].pulses>>16);
      f[8]=(uint8_t)(motion.move[i].pulses>>8); f[9]=(uint8_t)motion.move[i].pulses;
      f[10]=2U; f[11]=motion.sync; f[12]=0x6BU; ExchangeStart(f,13U,4U,0U); break;
    case TRIGGER:
      f[0]=0U; f[1]=0xFFU; f[2]=0x66U; f[3]=0x6BU;
      if (HAL_UART_Transmit(motor,f,4U,10U)!=HAL_OK) { Fail("synchronization transmit failed"); return; }
      motion.phase=WAIT_STATE; motion.next_at=now+100U; bus_ready=now+3U; break;
    case WAIT_STATE: f[1]=0x3AU; ExchangeStart(f,3U,4U,0U); break;
    default: break;
  }
}
static void ResetSpeeds(void) {
  default_rpm[0]=ARM_M5_DEFAULT_RPM;
  default_rpm[1]=ARM_M6_DEFAULT_RPM;
  default_rpm[2]=ARM_M7_DEFAULT_RPM;
}
static uint32_t PulseLimit(uint8_t i) {
  uint32_t limit=0U;
  (void)ArmUnits_ToPulses(i,max_travel[i],&limit);
  return limit;
}
static void ResetPresets(void) {
  fixed_travel[0]=ARM_M5_FIXED_TRAVEL_MILLI;
  fixed_travel[1]=ARM_M6_FIXED_TRAVEL_MILLI;
  fixed_travel[2]=ARM_M7_FIXED_TRAVEL_MILLI;
}
static void ShowPresets(void) {
  uint8_t i;
  for (i=0U;i<3U;++i) Log("Fixed M%u=%lu.%03lu %s; max=%lu.%03lu\r\n",(unsigned)(i+5U),
      (unsigned long)(fixed_travel[i]/1000U),(unsigned long)(fixed_travel[i]%1000U),i==0U?"deg":"mm",
      (unsigned long)(max_travel[i]/1000U),(unsigned long)(max_travel[i]%1000U));
}
static void ShowSpeeds(void) {
  Log("RPM M5=%lu M6=%lu M7=%lu (RAM)\r\n",
      (unsigned long)default_rpm[0],(unsigned long)default_rpm[1],(unsigned long)default_rpm[2]);
}
static void ExportConfig(void) {
  uint8_t i;
  Print("Update arm_config.h and rebuild:\r\n");
  for (i=0U;i<3U;++i) {
    const ArmUnitProfile *p=ArmUnits_Get(i);
    Log("#define ARM_M%u_CAL_PULSES %luU\r\n",(unsigned)(i+5U),(unsigned long)p->pulses);
    Log("#define ARM_M%u_CAL_TRAVEL_MILLI %luU\r\n",(unsigned)(i+5U),(unsigned long)p->travel_milli);
    Log("#define ARM_M%u_POS_DIR %uU\r\n",(unsigned)(i+5U),p->positive_direction);
    Log("#define ARM_M%u_DEFAULT_RPM %luU\r\n",(unsigned)(i+5U),(unsigned long)default_rpm[i]);
    Log("#define ARM_M%u_FIXED_TRAVEL_MILLI %luU\r\n",(unsigned)(i+5U),(unsigned long)fixed_travel[i]);
  }
}
static const char *UnitName(uint8_t i) { return i==0U?"deg":"mm"; }
static void ShowCalibration(void) {
  uint8_t i;
  for (i=0U;i<3U;++i) {
    const ArmUnitProfile *p=ArmUnits_Get(i);
    Log("CAL M%u: %lu pulses / %lu.%03lu %s; %s DIR=%u%s\r\n",(unsigned)(i+5U),
        (unsigned long)p->pulses,(unsigned long)(p->travel_milli/1000U),
        (unsigned long)(p->travel_milli%1000U),UnitName(i),
        i==0U?"CW":i==1U?"UP":"FORWARD",p->positive_direction,
        !p->pulses || !p->travel_milli?" (scale unset)":p->positive_direction>1U?" (direction unset)":"");
  }
  Print("RAM; save in arm_config.h.\r\n");
}
/* Maintenance is explicit: 'single' declares that only ONE driver is wired.
 * Identical replies from multiple same-ID drivers cannot be reliably detected. */
static void AddressFinish(const char *message) {
  address_op.phase=ADDR_IDLE;
  exchange.busy=exchange.done=exchange.raw=0U;
  Log("%s\r\n> ",message);
}
static void AddressStart(uint8_t changing,uint8_t old_id,uint8_t new_id) {
  uint8_t f[]={0U,0x15U,0x6BU};
  armed=0U; ClearPlan(); reference.valid=0U; reference.check=REF_UNCHECKED;
  address_op.changing=changing; address_op.old_id=old_id; address_op.new_id=new_id;
  address_op.phase=ADDR_DISCOVER;
  Print("USART6: ONE driver; arm locked.\r\n");
  ExchangeStart(f,sizeof(f),0U,1U);
}
static void AddressPoll(void) {
  uint8_t f[6],ok,phase;
  if (address_op.phase==ADDR_IDLE) return;
  if (exchange.done) {
    ok=exchange.ok; phase=(uint8_t)address_op.phase; exchange.done=exchange.raw=0U;
    if (phase==ADDR_DISCOVER) {
      if (!ok || exchange.length!=4U || exchange.reply[1]!=0x15U ||
          !exchange.reply[0] || exchange.reply[0]!=exchange.reply[2] || exchange.reply[3]!=0x6BU) {
        AddressFinish("ADDR FAILED: need single ID reply; no write.");
        return;
      }
      Log("ADDR FOUND: %u (%02X); ONE driver only.\r\n",
          exchange.reply[0],exchange.reply[0]);
      if (!address_op.changing) { AddressFinish("Scan done; ONE driver only."); return; }
      if (exchange.reply[0]!=address_op.old_id) {
        AddressFinish("ADDR FAILED: OLD ID mismatch; no write."); return;
      }
      if (address_op.old_id==address_op.new_id) { AddressFinish("ID unchanged; no write."); return; }
      address_op.phase=ADDR_STOP;
    } else if (phase==ADDR_STOP) {
      if (!ok) { AddressFinish("ADDR FAILED: no stop ACK; no write."); return; }
      address_op.phase=ADDR_WRITE;
    } else if (phase==ADDR_WRITE) {
      /* Some drivers reply using OLD, some NEW, and an ACK may be lost.
       * Do not retry the flash write. Probe the new ID in all uncertain cases. */
      if (ok && exchange.length==4U &&
          (exchange.reply[0]==address_op.old_id || exchange.reply[0]==address_op.new_id) &&
          exchange.reply[1]==0xAEU && exchange.reply[3]==0x6BU &&
          (exchange.reply[2]==0xE2U || exchange.reply[2]==0xEEU)) {
        AddressFinish("ADDR FAILED: driver rejected write."); return;
      }
      Print("SAVE=1 once; verify ID.\r\n");
      address_op.phase=ADDR_VERIFY;
    } else if (phase==ADDR_VERIFY) {
      if (!ok || exchange.reply[2]>1U) {
        AddressFinish("ADDR UNCONFIRMED: retry scan single."); return;
      }
      Log("ADDR VERIFIED: %u -> %u saved; reboot/scan.\r\n",
          address_op.old_id,address_op.new_id);
      AddressFinish("Done; arm locked."); return;
    }
    return;
  }
  if (exchange.busy || !Due(HAL_GetTick(),bus_ready)) return;
  f[0]=address_op.old_id;
  switch (address_op.phase) {
    case ADDR_STOP:
      f[1]=0xFEU; f[2]=0x98U; f[3]=0U; f[4]=0x6BU; ExchangeStart(f,5U,4U,0U); break;
    case ADDR_WRITE:
      f[1]=0xAEU; f[2]=0x4BU; f[3]=1U; f[4]=address_op.new_id; f[5]=0x6BU;
      ExchangeStart(f,6U,0U,1U); break;
    case ADDR_VERIFY:
      f[0]=address_op.new_id; f[1]=0x36U; f[2]=0x6BU; ExchangeStart(f,3U,8U,0U); break;
    default: break;
  }
}
static void ShowStatus(void) {
  uint8_t i;
  Log("arm state=%s armed=%u plan=%u\r\n",
      PhaseName(motion.phase),armed,plan_mask);
  if (address_op.phase!=ADDR_IDLE) Log("addr phase=%u %u->%u\r\n",
      (unsigned)address_op.phase,address_op.old_id,address_op.new_id);
  for (i=0U;i<3U;++i) {
    Log("M%u %s PPR=%lu cap=%lu RPMmax=%lu flags=%02X\r\n",
      (unsigned)(i+5U),axis_names[i],(unsigned long)(full_steps[i]*microsteps[i]),
      (unsigned long)PulseLimit(i),(unsigned long)max_rpm[i],motion.flags[i]);
    /* Diagnostics only: last successfully sampled motor position, retained
     * through a stop/fault. No UART query, motion or fault clearing here.
     * DecodePosition guarantees each signed magnitude fits uint32_t. */
    if (motion.sampled&(1U<<i)) {
      int64_t b=motion.baseline[i],p=motion.position[i];
      Log(" cached pulses=%lu RPM=%lu base=%c%lu pos=%c%lu counts\r\n",
        (unsigned long)motion.move[i].pulses,(unsigned long)motion.move[i].rpm,
        b<0?'-':'+',(unsigned long)(b<0?-b:b),p<0?'-':'+',(unsigned long)(p<0?-p:p));
    }
    if (plan_mask&(1U<<i)) Log(" plan D=%u P=%lu RPM=%lu\r\n",plan[i].direction,
      (unsigned long)plan[i].pulses,(unsigned long)plan[i].rpm);
  }
  if (motion.reason[0]) Log("last stop/fault: %s\r\n",motion.reason);
  ShowReference();
  ShowCalibration();
  ShowSpeeds();
  ShowPresets();
}
static void StartMotion(const Move requested[3],uint8_t mask,uint8_t sync) {
  uint8_t i; uint64_t duration=0U;
  if (!armed || motion.phase==FAULT) { Print("Locked/fault; clearfault, on.\r\n"); return; }
  for (i=0U;i<3U;++i) if (mask&(1U<<i)) {
    uint64_t ppr=(uint64_t)full_steps[i]*microsteps[i],t;
    if (!ppr || (full_steps[i]!=200U && full_steps[i]!=400U) || microsteps[i]>256U ||
        !requested[i].rpm || requested[i].rpm>max_rpm[i] || requested[i].pulses>PulseLimit(i)) {
      Print("Invalid profile/move limits.\r\n"); return;
    }
    t=(uint64_t)requested[i].pulses*60000U/(requested[i].rpm*ppr)*3U+10000U;
    if (t>duration) duration=t;
  }
  if (duration>ARM_MAX_MOVE_TIME_MS) { Print("Move too slow/long.\r\n"); return; }
  memset(&motion,0,sizeof(motion)); memcpy(motion.move,requested,sizeof(motion.move));
  reference.check=REF_UNCHECKED; lift_report=HAL_GetTick()-200U;
  motion.mask=mask; motion.sync=sync; speed_mask=0U; speed_report=HAL_GetTick()-200U;
  while (!(mask&(1U<<motion.index))) ++motion.index;
  motion.phase=PREFLIGHT_STOP; motion.deadline=HAL_GetTick()+(uint32_t)duration;
  ClearPlan(); Print("START: preflight; ! stops.\r\n");
}
static void Query(uint8_t addr,const char *name) {
  static const char *names[]={"version","voltage","pos","error","state","config","options"};
  static const uint8_t codes[]={0x1FU,0x24U,0x36U,0x37U,0x3AU,0x42U,0x1AU};
  uint8_t i,f[4]={addr,0U,0x6BU,0x6BU};
  for (i=0U;i<7U;++i) if (!strcmp(name,names[i])) {
    f[1]=codes[i];
    if (codes[i]==0x42U) f[2]=0x6CU;
    ExchangeStart(f,codes[i]==0x42U?4U:3U,0U,1U); return;
  }
  Print("Invalid read type.\r\n");
}
/* All physical moves are RELATIVE. The existing pulse/RPM caps and motion
 * preflight still apply. Preview and calibration never send motor commands. */
static uint8_t ProcessUnits(char **args,uint8_t count) {
  uint32_t id,pulses,amount,rpm,back; uint8_t i,negative,offset=0U;
  const ArmUnitProfile *profile; Move requested[3]={{0}};
  if (!strcmp(args[0],"cal")) {
    if (count==2U && !strcmp(args[1],"show")) { ShowCalibration(); return 1U; }
    if (count==2U && !strcmp(args[1],"defaults")) {
      ArmUnits_Init(); ClearPlan(); armed=0U; reference.valid=0U; reference.check=REF_UNCHECKED; ShowCalibration(); return 1U;
    }
    if (count==5U && !strcmp(args[1],"set") && Number(args[2],5U,7U,&id) &&
        Number(args[3],1U,1000000000U,&pulses) && ArmUnits_ParseMilli(args[4],&amount) &&
        ArmUnits_SetScale((uint8_t)(id-5U),pulses,amount)) {
      ClearPlan(); armed=0U; reference.valid=0U; reference.check=REF_UNCHECKED; Print("RAM cal; plan cleared; locked.\r\n");
      ShowCalibration(); return 1U;
    }
    if (count==4U && !strcmp(args[1],"dir") && Number(args[2],5U,7U,&id) &&
        Number(args[3],0U,1U,&pulses) && ArmUnits_SetDirection((uint8_t)(id-5U),(uint8_t)pulses)) {
      ClearPlan(); armed=0U; reference.valid=0U; reference.check=REF_UNCHECKED; Print("RAM dir; plan cleared; locked.\r\n");
      ShowCalibration(); return 1U;
    }
    Print("Use cal show/defaults/set ID P U/dir ID D.\r\n"); return 1U;
  }
  if (!strcmp(args[0],"convert")) {
    if (count!=4U || !Number(args[1],5U,7U,&id)) { Print("convert ID deg/mm/pulses VALUE\r\n"); return 1U; }
    i=(uint8_t)(id-5U);
    if (!strcmp(args[2],"pulses") && Number(args[3],1U,UINT32_MAX,&pulses) &&
        ArmUnits_FromPulses(i,pulses,&amount)) {
      Log("M%u: %lu pulses = %lu.%03lu %s (preview).\r\n",(unsigned)id,
          (unsigned long)pulses,(unsigned long)(amount/1000U),(unsigned long)(amount%1000U),UnitName(i));
      return 1U;
    }
    if (!strcmp(args[2],UnitName(i)) && ArmUnits_ParseMilli(args[3],&amount) && amount &&
        ArmUnits_ToPulses(i,amount,&pulses)) {
      Log("M%u: %lu.%03lu %s -> %lu pulses%s (preview).\r\n",(unsigned)id,
          (unsigned long)(amount/1000U),(unsigned long)(amount%1000U),UnitName(i),
          (unsigned long)pulses,amount>max_travel[i]?" (above single-move pulse cap)":!pulses?" (below one-pulse resolution)":"");
      return 1U;
    }
    Print("Invalid units/scale/overflow.\r\n"); return 1U;
  }
  if (!strcmp(args[0],"plan") && count>=2U) offset=1U;
  if (!strcmp(args[offset],"rotate")) i=0U;
  else if (!strcmp(args[offset],"lift")) i=1U;
  else if (!strcmp(args[offset],"rail")) i=2U;
  else return 0U;
  if (count!=(uint8_t)(offset+2U) && count!=(uint8_t)(offset+3U) && count!=(uint8_t)(offset+4U)) {
    Print("[plan] rotate/lift/rail DIR [VALUE [RPM]]\r\n"); return 1U;
  }
  if (!strcmp(args[offset+1U],i==0U?"cw":i==1U?"up":"forward")) negative=0U;
  else if (!strcmp(args[offset+1U],i==0U?"ccw":i==1U?"down":"backward")) negative=1U;
  else { Print("Invalid direction.\r\n"); return 1U; }
  profile=ArmUnits_Get(i); rpm=default_rpm[i];
  amount=fixed_travel[i];
  if ((count>offset+2U && !ArmUnits_ParseMilli(args[offset+2U],&amount)) || !amount || amount>max_travel[i] ||
      (count==(uint8_t)(offset+4U) && !Number(args[offset+3U],1U,max_rpm[i],&rpm)) ||
      !rpm || rpm>max_rpm[i]) {
    Print("Invalid travel/RPM; max 3 decimals.\r\n"); return 1U;
  }
  if (!profile->pulses || !profile->travel_milli || profile->positive_direction>1U) {
    Print("Uncalibrated: cal set/dir.\r\n"); return 1U;
  }
  if (!ArmUnits_ToPulses(i,amount,&pulses) || !pulses || pulses>PulseLimit(i)) {
    Print("Pulses zero/overflow/over cap.\r\n"); return 1U;
  }
  if (!ArmUnits_FromPulses(i,pulses,&back)) { Print("Conversion overflow.\r\n"); return 1U; }
  requested[i].pulses=pulses; requested[i].rpm=rpm;
  requested[i].direction=(uint8_t)(profile->positive_direction^negative);
  Log("M%u %s: %lu.%03lu %s -> %lu pulses; quantized=%lu.%03lu %s, DIR=%u RPM=%lu\r\n",
      (unsigned)(i+5U),args[offset+1U],(unsigned long)(amount/1000U),(unsigned long)(amount%1000U),UnitName(i),
      (unsigned long)pulses,(unsigned long)(back/1000U),(unsigned long)(back%1000U),UnitName(i),
      requested[i].direction,(unsigned long)rpm);
  if (offset) {
    if (motion.phase==FAULT) { Print("Fault: plan rejected.\r\n"); return 1U; }
    plan[i]=requested[i]; plan_mask|=(uint8_t)(1U<<i); Print("STAGED; run to execute.\r\n");
  } else StartMotion(requested,(uint8_t)(1U<<i),0U);
  return 1U;
}
static void ProcessLine(void) {
  char *args[7],*p=line; uint8_t count=0U; uint32_t addr,dir,pulses,rpm,new_id; Move requested[3]={{0}};
  while (*p) {
    while (*p==' ' || *p=='\t') ++p;
    if (!*p) break;
    if (count==7U) { Print("Too many arguments.\r\n"); return; }
    args[count++]=p;
    while (*p && *p!=' ' && *p!='\t') { if (*p>='A' && *p<='Z') *p=(char)(*p-'A'+'a'); ++p; }
    if (*p) *p++='\0';
  }
  if (!count) return;
  if (count==1U && (!strcmp(args[0],"stopall") || !strcmp(args[0],"stop"))) { AbortMotion("operator stop",0U); return; }
  if (count==1U && !strcmp(args[0],"status")) { ShowStatus(); return; }
  if (count==2U && !strcmp(args[0],"ref") && !strcmp(args[1],"status")) { ShowReference(); return; }
  if (count==1U && !strcmp(args[0],"help")) {
    Print("on/off | rotate/lift/rail | move/plan | run\r\nref set/status/check | read/cal/speed/preset/convert/config/status/stopall/clearfault\r\naddr scan single | addr set OLD NEW single: ONE driver!\r\n"); return;
  }
  if (ArmBench_IsBusy()) {
    Print("Busy: status/stopall.\r\n"); return;
  }
  if (count==2U && !strcmp(args[0],"ref") && !strcmp(args[1],"set")) {
    if (ArmBench_HasFault()) { Print("ARM REF blocked: fault.\r\n"); return; }
    reference.lift_down_sign=0;
    reference.mode=REF_CAPTURE; reference.axis=reference.valid=0U; reference.check=REF_UNCHECKED;
    ClearPlan(); Print("ARM REF sampling stationary M5/M6/M7.\r\n"); return;
  }
  if (count==2U && !strcmp(args[0],"ref") && !strcmp(args[1],"check")) {
    if (!ArmBench_CheckRouteStart()) Print("ARM REF check requires a recorded zero and idle, fault-free arm.\r\n");
    return;
  }
  if (count==2U && !strcmp(args[0],"config") && !strcmp(args[1],"show")) {
    ShowCalibration(); ShowSpeeds(); ShowPresets(); return;
  }
  if (count==2U && !strcmp(args[0],"config") && !strcmp(args[1],"export")) {
    ExportConfig(); return;
  }
  if (!strcmp(args[0],"preset")) {
    if (count==2U && !strcmp(args[1],"show")) { ShowPresets(); return; }
    if (count==2U && !strcmp(args[1],"defaults")) {
      ResetPresets(); ClearPlan(); ShowPresets(); return;
    }
    if (count==3U && Number(args[1],5U,7U,&addr) && ArmUnits_ParseMilli(args[2],&pulses) &&
        pulses && pulses<=max_travel[addr-5U]) {
      fixed_travel[addr-5U]=pulses; ClearPlan(); ShowPresets(); return;
    }
    Print("preset show/defaults/ID VALUE\r\n"); return;
  }
  if (!strcmp(args[0],"speed")) {
    uint8_t i;
    if (count==2U && !strcmp(args[1],"show")) { ShowSpeeds(); return; }
    if (count==2U && !strcmp(args[1],"defaults")) {
      ResetSpeeds(); ClearPlan(); ShowSpeeds(); return;
    }
    if (count==3U && !strcmp(args[1],"all") && Number(args[2],1U,3000U,&rpm)) {
      for (i=0U;i<3U;++i) if (rpm>max_rpm[i]) { Print("RPM outside axis limit.\r\n"); return; }
      for (i=0U;i<3U;++i) default_rpm[i]=rpm;
      ClearPlan(); ShowSpeeds(); return;
    }
    if (count==3U && Number(args[1],5U,7U,&addr) && Number(args[2],1U,max_rpm[addr-5U],&rpm)) {
      default_rpm[addr-5U]=rpm; ClearPlan(); ShowSpeeds(); return;
    }
    Print("speed show/defaults | speed 5/6/7/all RPM\r\n"); return;
  }
  if (ProcessUnits(args,count)) return;
  if (count==3U && !strcmp(args[0],"addr") && !strcmp(args[1],"scan") && !strcmp(args[2],"single")) {
    AddressStart(0U,0U,0U); return;
  }
  if (count==5U && !strcmp(args[0],"addr") && !strcmp(args[1],"set") &&
      Number(args[2],1U,255U,&addr) && Number(args[3],1U,255U,&new_id) && !strcmp(args[4],"single")) {
    AddressStart(1U,(uint8_t)addr,(uint8_t)new_id); return;
  }
  if (count==1U && !strcmp(args[0],"off")) { AbortMotion("arm off",0U); return; }
  if (count==1U && !strcmp(args[0],"clearfault")) {
    if (motion.phase==FAULT) {
      if (motion.stop_failed) { Print("Retry stopall; require all ACKs.\r\n"); return; }
      motion.phase=IDLE; armed=0U; Print("Fault cleared; arm locked.\r\n");
    } else Print("No latched fault.\r\n");
    return;
  }
  if (count==1U && !strcmp(args[0],"on")) {
    if (motion.phase==FAULT) Print("Fault latched; inspect then clearfault.\r\n");
    else { armed=1U; Print("Armed; no homing/limit protection.\r\n"); }
    return;
  }
  if (count==2U && !strcmp(args[0],"plan") && !strcmp(args[1],"clear")) { ClearPlan(); Print("Plan cleared.\r\n"); return; }
  if (count==3U && !strcmp(args[0],"read") && Number(args[1],1U,255U,&addr)) {
    Query((uint8_t)addr,args[2]); return;
  }
  if (count==1U && !strcmp(args[0],"run")) {
    uint8_t i; uint32_t total=0U;
    if (!plan_mask) { Print("No plan.\r\n"); return; }
    memcpy(requested,plan,sizeof(requested));
    for (i=0U;i<3U;++i) { total+=requested[i].pulses; if (!requested[i].rpm) requested[i].rpm=default_rpm[i]; }
    if (!total) { Print("Zero-motion plan.\r\n"); return; }
    /* Zero-motion preloads overwrite stale pending moves on unselected axes. */
    StartMotion(requested,7U,1U); return;
  }
  if ((count==4U || count==5U) && (!strcmp(args[0],"move") || !strcmp(args[0],"plan")) &&
      Number(args[1],5U,7U,&addr) && Number(args[2],0U,1U,&dir) &&
      Number(args[3],!strcmp(args[0],"move")?1U:0U,PulseLimit((uint8_t)(addr-5U)),&pulses)) {
    uint8_t i=(uint8_t)(addr-5U); rpm=default_rpm[i];
    if ((count==5U && !Number(args[4],1U,max_rpm[i],&rpm)) || !rpm || rpm>max_rpm[i]) { Print("RPM outside axis limit.\r\n"); return; }
    requested[i].pulses=pulses; requested[i].rpm=rpm; requested[i].direction=(uint8_t)dir;
    if (!strcmp(args[0],"plan")) {
      if (motion.phase==FAULT) { Print("Fault: plan rejected.\r\n"); return; }
      plan[i]=requested[i]; plan_mask|=(uint8_t)(1U<<i);
      Log("STAGED M%u dir=%u pulses=%lu rpm=%lu.\r\n",
        (unsigned)addr,(unsigned)dir,(unsigned long)pulses,(unsigned long)rpm);
    } else StartMotion(requested,(uint8_t)(1U<<i),0U);
    return;
  }
  Print("Invalid command/args; see manual.\r\n");
}

void ArmBench_Init(UART_HandleTypeDef *c,UART_HandleTypeDef *m) {
  (void)c; motor=m; Emm_Init(&bus,m,ARM_REPLY_TIMEOUT_MS);
  memset(&motion,0,sizeof(motion)); ClearPlan(); speed_mask=0U; speed_report=0U;
  memset(&address_op,0,sizeof(address_op));
  memset(&reference,0,sizeof(reference));
  ArmUnits_Init(); ResetSpeeds(); ResetPresets(); armed=0U;
}
void ArmBench_Poll(void) { if (motor_error && Busy() && motion.phase!=ABORTING) { motor_error=0U; Fail("motor UART RX error/overflow"); }
  ExchangePoll();
  if (address_op.phase!=ADDR_IDLE) AddressPoll();
  else if (reference.mode!=REF_IDLE) ReferencePoll();
  else PrintRawReply();
  MotionPoll(); }
void ArmBench_Command(const char *command) {
  if (!command || strlen(command)>=sizeof(line)) { Print("Command too long.\r\n"); return; }
  memcpy(line,command,strlen(command)+1U); ProcessLine();
}
uint8_t ArmBench_IsBusy(void) { return (uint8_t)(Busy() || reference.mode!=REF_IDLE || address_op.phase!=ADDR_IDLE || exchange.busy || exchange.done || !Due(HAL_GetTick(),bus_ready)); }
uint8_t ArmBench_HasFault(void) { return (uint8_t)(motion.phase==FAULT || (motion.phase==ABORTING && motion.abort_fault)); }
uint8_t ArmBench_IsArmed(void) { return armed; }
const char *ArmBench_LivePhase(void) {
  if (reference.mode!=REF_IDLE) return "reference";
  if (address_op.phase!=ADDR_IDLE) return "address";
  if (motion.phase==WAIT_STATE || motion.phase==WAIT_POSITION) return "moving";
  return PhaseName(motion.phase);
}
void ArmBench_Stop(const char *reason,uint8_t fault) { AbortMotion(reason,fault); }
void ArmBench_OnRx(UART_HandleTypeDef *u) { if (u==motor) Emm_OnRx(&bus); }
void ArmBench_OnError(UART_HandleTypeDef *u) { if (u==motor) Emm_OnError(&bus); }

uint8_t ArmBench_CheckRouteStart(void) {
  if (!reference.valid || ArmBench_IsBusy() || ArmBench_HasFault()) return 0U;
  /* The physical M7 transport extension is checked by the operator.
   * Keep its captured zero for vision compensation and arm ref status. */
  reference.mode=REF_START; reference.axis=0U; reference.check=REF_CHECKING;
  reference.window.samples=0U;
  reference.next_at=HAL_GetTick();
  reference.check_until=reference.next_at+ARM_ROUTE_RECHECK_MS;
  return 1U;
}
uint8_t ArmBench_HasReference(void) { return reference.valid; }
uint8_t ArmBench_RouteReady(void) { return reference.check==REF_VERIFIED; }

uint8_t ArmBench_RouteAction(uint8_t axis,uint8_t negative,uint32_t amount,uint16_t rpm) {
  Move requested[3]={{0}}; const ArmUnitProfile *profile;
  if(axis>2U || negative>1U || amount>max_travel[axis] ||
      ArmBench_IsBusy() || ArmBench_HasFault() || !armed) return 0U;
  if (!amount) {
    if (axis!=1U || !reference.valid) return 0U;
    /* A zero-depth tray placement still needs a fresh, stable top reading. */
    uint32_t now=HAL_GetTick();
    memset(&motion,0,sizeof(motion));
    motion.mask=2U;
    motion.index=1U;
    motion.phase=CHECK_LIFT;
    motion.next_at=now;
    motion.check_until=now+ARM_ROUTE_RECHECK_MS;
    motion.deadline=motion.check_until+ARM_REPLY_TIMEOUT_MS;
    lift_report=now-200U;
    return 1U;
  }
  profile=ArmUnits_Get(axis);
  requested[axis].rpm=rpm?rpm:default_rpm[axis];
  if(!profile || profile->positive_direction>1U || !ArmUnits_ToPulses(axis,amount,&requested[axis].pulses) ||
      !requested[axis].pulses || requested[axis].pulses>PulseLimit(axis)) return 0U;
  requested[axis].direction=profile->positive_direction^negative;
  StartMotion(requested,(uint8_t)(1U<<axis),0U);
  if (motion.phase!=PREFLIGHT_STOP) return 0U;
  motion.route_action=(uint8_t)(reference.valid && axis<2U);
  motion.returning=axis==0U?negative:(uint8_t)!negative;
  return 1U;
}

uint8_t ArmBench_RouteRail(int16_t from_mm,int16_t to_mm) {
  uint32_t from,to;
  int32_t delta=(int32_t)to_mm-from_mm;
  if (!reference.valid || from_mm<45 || from_mm>85 || to_mm<45 || to_mm>85 ||
      !ArmUnits_ToPulses(2U,(uint32_t)from_mm*1000U,&from) ||
      !ArmUnits_ToPulses(2U,(uint32_t)to_mm*1000U,&to) ||
      !ArmBench_RouteAction(2U,delta<0,(uint32_t)(delta<0?-delta:delta)*1000U,5U)) return 0U;
  motion.route_action=2U;
  motion.rail_from=EncoderTravel(2U,from);
  motion.rail_to=EncoderTravel(2U,to);
  return 1U;
}

void ArmBench_ResetPickupRail(void) { pickup_forward_sign=0; }

uint8_t ArmBench_PickupRail(int16_t from_offset_mm,int16_t to_offset_mm) {
  int32_t delta=(int32_t)to_offset_mm-from_offset_mm;
  if (!reference.valid || from_offset_mm < -ARM_PICKUP_RAIL_LIMIT_MM || from_offset_mm > ARM_PICKUP_RAIL_LIMIT_MM ||
      to_offset_mm < -ARM_PICKUP_RAIL_LIMIT_MM || to_offset_mm > ARM_PICKUP_RAIL_LIMIT_MM ||
      !ArmBench_RouteAction(2U,delta<0,(uint32_t)(delta<0?-delta:delta)*1000U,5U)) return 0U;
  motion.route_action=PICKUP_RAIL_ACTION;
  motion.pickup_offset_mm=to_offset_mm;
  return 1U;
}
