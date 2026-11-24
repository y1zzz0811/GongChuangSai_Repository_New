#include "route2.h"
#include "app_console.h"
#include "competition.h"
#include "arm_bench.h"
#include "arm_config.h"
#include "route_config.h"
#include "mission.h"
#include "motor_bench.h"
#include "qr_bridge.h"
#include "route_scene.h"
#include "servo.h"
#include <string.h>

#define COARSE_PITCH_MM 150
#define SERVO_SETTLE_MS 300U
#define CONFIRM_TIMEOUT_MS 30000U
#define COMMISSIONING_LIMIT_MS 1800000U
#define REQUEST_LIMIT_MS 120000U
#define JOINT_RAW_TRIAL_LIMIT_MS 480000U
#define FORWARD_TOLERANCE_UM 5000
#define RAW_FORWARD_TOLERANCE_UM FORWARD_TOLERANCE_UM
#define LEFT_TOLERANCE_UM FORWARD_TOLERANCE_UM
#define ALIGN_MAX_STEPS 4U
#define SINGLE_SHIFT_LIMIT_MM 100U
#define RAW_SHIFT_LIMIT_MM SINGLE_SHIFT_LIMIT_MM
#define PICKUP_TRAVEL_LIMIT_MM 30U
#define PICKUP_RAIL_TARGET_MM 10
#define PICKUP_TRIAL 2U
#define RING_TRIAL 3U
#define STACK_TRIAL 4U
#define RAIL_NOMINAL_MM ((int16_t)(ARM_ROUTE_RAIL_MILLI/1000U))

typedef enum { QR, BASE_SHIFT, BASE_ACK, DRIVE, HOME_ACK, ALIGN, TRAY,
               WORK, RETURN_CHECK, ITEM_ACK, OBSERVE_TURN, OBSERVE, WAIT_RING,
               SHIFT_TURN, SHIFT_MOVE, RESTORE_TURN, TRIAL_ACK, WAIT_RAW,
               FIRST_ITEM_CHECK,PICKUP_OBSERVE,PICKUP_RESULT,PICKUP_RAIL,RAW_ADJUST,
               PICKUP_COARSE } Phase;
typedef struct { uint8_t batch,color; } Item; /* color=0 means empty */
static Item tray[3],rough[3],temp[3];
static uint32_t confirmation_seq,confirmation_at,stable_at;
static uint8_t pending,active,awaiting,answer,work_index,ready_confirmed,settling;
static RouteStation station,destination;
static int16_t station_offset;
static const char *const transfer_names[]={
  "none","face_station","open_claw","lower_pick","grip","lift_pick",
  "face_tray","lower_place","release","lift_place","complete","rail_apply","rail_restore"
};
static Phase phase;
static uint8_t trial;
/* Route3 reuses every mission node; only M6 motion execution is omitted.
 * Its encoder, fixed-top clearance checks and stop ACK stay mandatory. */
static uint8_t no_lift,route_id;
static uint8_t single_test;
static const char *single_kind;
static uint8_t pickup_budget,pickup_travel,pickup_aligned;
static int16_t pickup_rail_target;
static uint8_t joint_pickup,forward_budget,forward_used;
static int16_t forward_net;
static uint32_t trial_started;
static TransferStep current_step;
static uint8_t vision,raw_vision,rough_budget,temp_budget,rail_budget,adjustments,rail_step,shift_travel;
static uint8_t raw_shift_budget;
static uint8_t formal_forward_budget;
static int16_t raw_left_mm;
static int16_t coarse_offset,shift_mm,rail_offset;
static const char *confirmation_kind;
static MissionRequest confirmation_request;
static const char *current_operation;
static MissionRequest current_request;
static uint32_t last_decision;
static const char *const operation_names[]={
  "go_qr","read_qr","go_raw","load_raw","go_rough","place_rough",
  "reload_rough","go_temp","place_temp","go_home"
};

static uint8_t Loading(const MissionRequest *r) {
  return r->op==LOAD_RAW || r->op==RELOAD_ROUGH;
}
static uint8_t VisualItem(const MissionRequest *r) {
  return trial>=RING_TRIAL ||
    (!trial && vision && (r->op==PLACE_ROUGH || r->op==RELOAD_ROUGH || r->op==PLACE_TEMP));
}
static uint8_t RelativeVisionRail(void) {
  return trial>=PICKUP_TRIAL || (trial && single_test && current_request.op==LOAD_RAW);
}
static uint8_t MoveVisionRail(int16_t from,int16_t to) {
  return RelativeVisionRail()?ArmBench_PickupRail(from,to):
    ArmBench_RouteRail((int16_t)(RAIL_NOMINAL_MM+from),(int16_t)(RAIL_NOMINAL_MM+to));
}
static uint8_t TopTarget(const MissionRequest *r) {
  return r->op==RELOAD_ROUGH || r->stack;
}
static uint8_t SameItem(Item item,const MissionRequest *r) {
  return item.color==r->color && item.batch==r->batch;
}
static uint8_t TrayFull(uint8_t batch) {
  unsigned i;
  for(i=0;i<3U;++i) if (!tray[i].color || tray[i].batch!=batch) return 0U;
  return 1U;
}
static uint8_t TrayEmpty(void) {
  return !tray[0].color && !tray[1].color && !tray[2].color;
}
static uint32_t Ready(void) {
  /* These ports use the operator as the observation source in commissioning. */
  return active && RouteScene_IsClear() && ArmBench_IsArmed() &&
    !ArmBench_HasFault() && !MotorBench_HasFault() && !Servo_HasFault() ? SERVICE_ALL : 0U;
}
static uint8_t DevicesIdle(void) {
  return !MotorBench_IsBusy() && !ArmBench_IsBusy() && !Servo_IsBusy();
}
static uint8_t Settled(void) {
  if (!DevicesIdle()) return 0U;
  if (!settling) return 1U;
  if (!stable_at) { stable_at=HAL_GetTick(); return 0U; }
  if (HAL_GetTick()-stable_at<SERVO_SETTLE_MS) return 0U;
  settling=0U;
  return 1U;
}
static void StartSettling(void) { settling=1U; stable_at=0U; }

static void ReportConfirmation(const MissionRequest *r) {
  App_Log("@route2 %lu %s %s %u %u %u %u %u %u\r\n",
    (unsigned long)confirmation_seq,confirmation_kind,trial>=PICKUP_TRIAL?current_operation:operation_names[r->op],
    r->batch+1U,r->index+1U,r->color,r->op==LOAD_RAW?0U:r->ring,r->stack?2U:1U,r->index+1U);
}
static PortResult Confirm(const MissionRequest *r,const char *kind) {
  if (!awaiting) {
    if (++confirmation_seq==0U) ++confirmation_seq;
    confirmation_kind=kind; awaiting=1U; answer=0U; confirmation_at=HAL_GetTick();
    confirmation_request=*r;
    ReportConfirmation(r);
    return PORT_WAIT;
  }
  if (answer==2U || HAL_GetTick()-confirmation_at>=CONFIRM_TIMEOUT_MS) return PORT_ERROR;
  if (!answer) return PORT_WAIT;
  awaiting=answer=0U;
  return PORT_DONE;
}

static uint8_t CanTravel(void) {
  if (station==ROUTE_START || station==ROUTE_SCAN) return ArmBench_HasReference();
  return ArmBench_RouteReady();
}
static PortResult StartLeg(void) {
  if (!DevicesIdle() || !CanTravel() || station_offset ||
      !RouteScene_AllowLeg(station,destination)) return PORT_ERROR;
  if (!MotorBench_StartLeg(station,destination)) return PORT_ERROR;
  phase=DRIVE;
  return PORT_WAIT;
}
static PortResult BeginLeg(const MissionRequest *r,RouteStation to) {
  uint8_t carrying=r->op==GO_ROUGH || r->op==GO_TEMP;
  if (!DevicesIdle() || !CanTravel() ||
      !(carrying?TrayFull(r->batch):TrayEmpty()) ||
      !RouteScene_AllowLeg(station,to)) return PORT_ERROR;
  destination=to;
  if (station==ROUTE_START || station==ROUTE_SCAN) return StartLeg();
  if (station_offset && !MotorBench_Shift((int16_t)-station_offset)) return PORT_ERROR;
  phase=BASE_SHIFT;
  return PORT_WAIT;
}

static PortResult BeginItem(const MissionRequest *r) {
  int16_t offset=0;
  uint8_t loading=Loading(r);
  uint8_t first=r->op==LOAD_RAW && r->batch==0U && r->index==0U;
  Item slot;
  if (!DevicesIdle() || (!first && !ArmBench_RouteReady()) || r->index>2U || !r->color) return PORT_ERROR;
  if (station!=(r->op==LOAD_RAW?ROUTE_RAW:r->op==PLACE_TEMP?ROUTE_TEMP:ROUTE_ROUGH)) return PORT_ERROR;
  slot=tray[r->index];
  if (loading?slot.color!=0U:!SameItem(slot,r)) return PORT_ERROR;
  if (station!=ROUTE_RAW) {
    if (r->ring<1U || r->ring>3U) return PORT_ERROR;
    if (r->op==PLACE_ROUGH && rough[r->ring-1U].color) return PORT_ERROR;
    if (r->op==RELOAD_ROUGH &&
        (!SameItem(rough[r->ring-1U],r) ||
         (r->index==0U && (!rough[0].color || !rough[1].color || !rough[2].color)))) return PORT_ERROR;
    if (r->op==PLACE_TEMP) {
      Item base=temp[r->ring-1U];
      if (r->stack?(base.color!=r->color || base.batch!=0U):base.color!=0U) return PORT_ERROR;
    }
    /* Both stations have 3,2,1 in ascending map coordinates.
     * Vehicle right is +map x at D and +map y at E. */
    offset=(int16_t)(((int)2-r->ring)*COARSE_PITCH_MM);
  }
  if (offset!=station_offset && !MotorBench_Shift((int16_t)(offset-station_offset))) return PORT_ERROR;
  station_offset=offset;
  coarse_offset=offset; rail_offset=0; adjustments=rail_step=shift_travel=0U;
  raw_left_mm=0;
  joint_pickup=1U;
  forward_net=0;
  forward_budget=formal_forward_budget; forward_used=0U;
  pickup_budget=station==ROUTE_RAW?raw_shift_budget:station==ROUTE_ROUGH?rough_budget:temp_budget;
  pickup_travel=0U;
  pickup_aligned=0U;
  work_index=ready_confirmed=0U;
  current_step=WORK_NONE;
  if (first && !ArmBench_CheckRouteStart()) return PORT_ERROR;
  phase=first?FIRST_ITEM_CHECK:ALIGN;
  return PORT_WAIT;
}
static PortResult Begin(const MissionRequest *r) {
  current_operation=operation_names[r->op];
  current_request=*r;
  if (r->home!=1U) return PORT_ERROR;
  switch(r->op) {
    case GO_QR: return BeginLeg(r,ROUTE_SCAN);
    case READ_QR: phase=QR; return PORT_WAIT;
    case GO_RAW: return BeginLeg(r,ROUTE_RAW);
    case GO_ROUGH: return BeginLeg(r,ROUTE_ROUGH);
    case GO_TEMP: return BeginLeg(r,ROUTE_TEMP);
    case GO_HOME: return BeginLeg(r,ROUTE_HOME);
    default: return BeginItem(r);
  }
}
static PortResult Work(const MissionRequest *r) {
  RouteWorkStep action;
  if (!Settled()) return PORT_WAIT;
  /* Raw pickup waits with the claw open facing the turntable. Vision
   * transfers ask the operator before moving into the observation pose. */
  uint8_t visual=VisualItem(r),loading=Loading(r);
  TransferStep step=RouteLibrary_Transfer(station,loading,
      r->stack?RouteLibrary_StackDepth():0U,work_index,&action);
  TransferStep ready_step=loading?(visual?FACE_STATION:LOWER_PICK):LOWER_PICK;
  if (step==WORK_NONE) return PORT_ERROR;
  if (trial==PICKUP_TRIAL && step==LOWER_PICK && !pickup_aligned) {
    phase=PICKUP_OBSERVE;
    return PORT_WAIT;
  }
  if (!ready_confirmed && step==ready_step) {
    PortResult result=Confirm(r,"ready");
    if (result!=PORT_DONE) return result;
    ready_confirmed=1U;
    if (visual) {
      phase=OBSERVE_TURN;
      return PORT_WAIT;
    }
  }
  if (r->op==LOAD_RAW && step==LOWER_PICK && !pickup_aligned &&
      (single_test || (!trial && vision && raw_vision))) {
    if (!QrBridge_RequestRaw(r->color,(int16_t)(forward_net-rail_offset),raw_left_mm,
                             adjustments!=0U)) return PORT_ERROR;
    phase=WAIT_RAW;
    return PORT_WAIT;
  }
  if (loading && step==FACE_TRAY && rail_offset && !rail_step) {
    /* Lift clear before restoring M7, then turn back to the tray. */
    if (!MoveVisionRail(rail_offset,0)) return PORT_ERROR;
    rail_step=1U;
    current_step=RAIL_RESTORE;
    StartSettling();
    return PORT_WAIT;
  }
  if (visual && !loading && (step==LOWER_PLACE || step==FACE_TRAY) && !rail_step) {
    /* M5 faces the target at step 4 and remains there until step 7. */
    if (rail_offset && !MoveVisionRail(step==LOWER_PLACE?0:rail_offset,
                                      step==LOWER_PLACE?rail_offset:0)) return PORT_ERROR;
    rail_step=1U;
    if (rail_offset) current_step=step==LOWER_PLACE?RAIL_APPLY:RAIL_RESTORE;
    StartSettling();
    /* Reuse the empty-claw alignment; do not observe an occluded target
     * while carrying material. The saved M7 correction is applied above. */
    return PORT_WAIT;
  }
  if (step==WORK_COMPLETE) {
    if (!ArmBench_CheckRouteStart()) return PORT_ERROR;
    phase=RETURN_CHECK;
    return PORT_WAIT;
  }
  if (action.claw) {
    if (!Servo_Claw(action.claw==2U)) return PORT_ERROR;
    StartSettling();
  }
  if (no_lift && action.axis==1U) {
    App_Log("ROUTE3 skip M6 %s; simulated transfer.\r\n",transfer_names[step]);
  } else if (action.axis!=255U && !ArmBench_RouteAction(action.axis,action.negative,action.amount,action.rpm)) return PORT_ERROR;
  current_step=step;
  ++work_index;
  rail_step=0U;
  return PORT_WAIT;
}

static uint8_t ObserveTurn(uint8_t back) {
  RouteWorkStep turn;
  TransferStep wanted=back?FACE_TRAY:FACE_STATION;
  for (uint8_t i=0U;;++i) {
    TransferStep step=RouteLibrary_Transfer(station,0U,0U,i,&turn);
    if (step==WORK_NONE || step==WORK_COMPLETE) return 0U;
    if (step==wanted)
      return turn.axis==0U && ArmBench_RouteAction(turn.axis,turn.negative,turn.amount,turn.rpm);
  }
}

static PortResult ApplyPickupRail(void) {
  if (pickup_rail_target!=rail_offset) {
    if (!MoveVisionRail(rail_offset,pickup_rail_target)) {
      App_Print("PICKUP TEST M7 correction blocked by travel or reference.\r\n");
      return PORT_ERROR;
    }
    rail_offset=pickup_rail_target;
    App_Log("@r2choice rail %d\r\n",(int)rail_offset);
    StartSettling();
  }
  phase=trial?PICKUP_OBSERVE:OBSERVE;
  return PORT_WAIT;
}

/* Chassis motion always gets a new observation before M7 fine correction. */
static PortResult CorrectJointPickup(int32_t forward_um,int32_t left_um,int32_t forward_tolerance_um) {
  int32_t forward=forward_um/1000;
  int32_t right=(left_um < -LEFT_TOLERANCE_UM || left_um > LEFT_TOLERANCE_UM)?-left_um/1000:0;
  int32_t next_rail=rail_offset-
    ((forward_um < -forward_tolerance_um || forward_um > forward_tolerance_um)?forward:0);
  const int32_t rail_limit=trial?ARM_PICKUP_RAIL_LIMIT_MM:rail_budget;
  int32_t longitudinal=0;
  uint32_t lateral_travel=(uint32_t)(right<0?-right:right);
  uint8_t raw=(uint8_t)(current_request.op==LOAD_RAW);

  if (adjustments>=ALIGN_MAX_STEPS) {
    App_Print("PICKUP TEST alignment did not converge.\r\n");
    return PORT_ERROR;
  }
  if (pickup_travel+lateral_travel>pickup_budget) {
    App_Log("PICKUP TEST chassis travel budget exceeded: need %lu mm, remaining %u mm (limit %u mm).\r\n",
            (unsigned long)lateral_travel,(unsigned)(pickup_budget-pickup_travel),(unsigned)pickup_budget);
    return PORT_ERROR;
  }
  if (next_rail < -rail_limit || next_rail > rail_limit) {
    int32_t goal=rail_limit<PICKUP_RAIL_TARGET_MM?rail_limit:PICKUP_RAIL_TARGET_MM;
    if(next_rail<0) goal=-goal;
    int32_t remaining=(int32_t)forward_budget-forward_used;
    longitudinal=goal-next_rail;
    if (longitudinal>remaining) longitudinal=remaining;
    if (longitudinal < -remaining) longitudinal=-remaining;
    if (!longitudinal || next_rail+longitudinal < -rail_limit || next_rail+longitudinal > rail_limit) {
      int32_t minimum=(next_rail<0?-next_rail:next_rail)-rail_limit;
      App_Log("PICKUP TEST forward chassis budget insufficient: need at least %ld mm %s, remaining %ld mm (M7 target %ld mm).\r\n",
              (long)minimum,next_rail<0?"forward":"backward",(long)remaining,(long)next_rail);
      return PORT_ERROR;
    }
  }
  if (longitudinal || right) {
    if(!trial) {
      /* Map-derived clearance to the restricted work zone. Expanded budgets
       * never authorize crossing it; restore fore/aft movement after each item. */
      int32_t clearance=station==ROUTE_RAW?(int32_t)ROUTE2_RAW_Y-80-150:
        station==ROUTE_ROUGH?2250-150-(int32_t)ROUTE2_ROUGH_Y:(int32_t)ROUTE2_TEMP_X-150-150;
      if((int32_t)forward_net+longitudinal < -clearance ||
          (!raw && ((int32_t)station_offset+right < -COARSE_PITCH_MM-(int32_t)SINGLE_SHIFT_LIMIT_MM ||
                    (int32_t)station_offset+right > COARSE_PITCH_MM+(int32_t)SINGLE_SHIFT_LIMIT_MM))) {
        App_Print("ROUTE2 correction blocked: work-zone boundary.\r\n"); return PORT_ERROR;
      }
    }
    if (!RouteScene_IsClear()) {
      App_Print("PICKUP TEST chassis correction blocked: scene not clear.\r\n");
      return PORT_ERROR;
    }
    if (longitudinal) {
      if (!MotorBench_TrimTravel((int16_t)longitudinal)) return PORT_ERROR;
      forward_used+=(uint8_t)(longitudinal<0?-longitudinal:longitudinal);
      forward_net=(int16_t)(forward_net+longitudinal);
      App_Log("@r2choice foreaft %d\r\n",(int)longitudinal);
    } else {
      if (!MotorBench_TrimShift((int16_t)right)) return PORT_ERROR;
      pickup_travel+=(uint8_t)lateral_travel;
      if (raw) raw_left_mm=(int16_t)(raw_left_mm-right);
      else if(!trial) station_offset=(int16_t)(station_offset+right);
      App_Log("@r2choice shift %d\r\n",(int)right);
    }
    StartSettling();
    ++adjustments;
    phase=PICKUP_COARSE;
    return PORT_WAIT;
  }
  ++adjustments;
  pickup_rail_target=(int16_t)next_rail;
  if (!raw) return ApplyPickupRail();
  phase=RAW_ADJUST;
  return PORT_WAIT;
}

static PortResult AlignSingle(void) {
  int32_t forward,left;
  int result=QrBridge_ReadRing(&forward,&left);
  if (!result) return PORT_WAIT;
  if (result<0) {
    App_Print("PICKUP TEST vision timeout.\r\n");
    return PORT_ERROR;
  }
  App_Log("ROUTE2 %s %u fwd=%ld left=%ld um\r\n",
    trial==PICKUP_TRIAL?"pickup":trial==STACK_TRIAL?"top":"ring",
    (unsigned)(trial==PICKUP_TRIAL?1U:current_request.ring),(long)forward,(long)left);
  if (forward>=-FORWARD_TOLERANCE_UM && forward<=FORWARD_TOLERANCE_UM &&
      left>=-LEFT_TOLERANCE_UM && left<=LEFT_TOLERANCE_UM) {
    if (trial==PICKUP_TRIAL) {
      pickup_aligned=1U;
      phase=WORK;
      return Work(&current_request);
    }
    if (rail_offset && !MoveVisionRail(rail_offset,0)) return PORT_ERROR;
    phase=RESTORE_TURN;
    App_Print("@r2choice aligned\r\n");
    return PORT_WAIT;
  }
  if (joint_pickup) return CorrectJointPickup(forward,left,FORWARD_TOLERANCE_UM);
  if (adjustments>=ALIGN_MAX_STEPS) {
    App_Log("PICKUP TEST alignment did not converge: fwd=%ld left=%ld um after %u corrections (limits %u/%u um).\r\n",
      (long)forward,(long)left,(unsigned)adjustments,(unsigned)FORWARD_TOLERANCE_UM,(unsigned)LEFT_TOLERANCE_UM);
    return PORT_ERROR;
  }
  int32_t right_mm=(left < -LEFT_TOLERANCE_UM || left > LEFT_TOLERANCE_UM)?-left/1000:0;
  uint32_t travel=(uint32_t)(right_mm<0?-right_mm:right_mm);
  /* M7 forward points toward the rear in the stationary observation pose. */
  int32_t next_rail=(int32_t)rail_offset-
    ((forward < -FORWARD_TOLERANCE_UM || forward > FORWARD_TOLERANCE_UM)?forward/1000:0);
  if (travel>(uint32_t)(pickup_budget-pickup_travel)) {
    App_Log("PICKUP TEST chassis travel budget exceeded: need %lu mm, remaining %u mm (limit %u mm).\r\n",
      (unsigned long)travel,(unsigned)(pickup_budget-pickup_travel),pickup_budget);
    return PORT_ERROR;
  }
  if (next_rail < -ARM_PICKUP_RAIL_LIMIT_MM || next_rail > ARM_PICKUP_RAIL_LIMIT_MM)
    return CorrectJointPickup(forward,left,FORWARD_TOLERANCE_UM);
  ++adjustments;
  pickup_rail_target=(int16_t)next_rail;
  if (!right_mm) return ApplyPickupRail();
  /* The empty gripper stays at the working top, facing the target. The
   * operator has confirmed clearance for this pose and lateral sweep. */
  if ((!single_test && !RouteScene_IsClear()) || !MotorBench_TrimShift((int16_t)right_mm)) return PORT_ERROR;
  pickup_travel=(uint8_t)(pickup_travel+travel);
  App_Log("@r2choice shift %d\r\n",(int)right_mm);
  StartSettling();
  phase=PICKUP_RAIL;
  return PORT_WAIT;
}

static PortResult AlignRaw(const MissionRequest *r) {
  int32_t forward,left,right_mm,next_rail,rail_limit;
  uint32_t travel;
  int result=QrBridge_ReadRing(&forward,&left);
  if (!result) return PORT_WAIT;
  if (result<0) {
    App_Print("ROUTE2 raw vision timeout or expired reply.\r\n");
    return PORT_ERROR;
  }
  App_Log("ROUTE2 raw %u fwd=%ld left=%ld um\r\n",(unsigned)r->color,(long)forward,(long)left);
  if (forward>=-RAW_FORWARD_TOLERANCE_UM && forward<=RAW_FORWARD_TOLERANCE_UM &&
      left>=-LEFT_TOLERANCE_UM && left<=LEFT_TOLERANCE_UM) {
    pickup_aligned=1U;
    phase=WORK;
    return Work(r);
  }
  if (joint_pickup) return CorrectJointPickup(forward,left,RAW_FORWARD_TOLERANCE_UM);
  if (adjustments>=ALIGN_MAX_STEPS) {
    App_Print("ROUTE2 raw alignment did not converge.\r\n");
    return PORT_ERROR;
  }
  right_mm=(left < -LEFT_TOLERANCE_UM || left > LEFT_TOLERANCE_UM)?-left/1000:0;
  travel=(uint32_t)(right_mm<0?-right_mm:right_mm);
  next_rail=(int32_t)rail_offset-
    ((forward < -RAW_FORWARD_TOLERANCE_UM || forward > RAW_FORWARD_TOLERANCE_UM)?forward/1000:0);
  rail_limit=RelativeVisionRail()?ARM_PICKUP_RAIL_LIMIT_MM:rail_budget;
  if (next_rail < -rail_limit || next_rail > rail_limit) {
    App_Log("ROUTE2 raw M7 correction exceeds budget: need %ld mm, limit %ld mm.\r\n",
            (long)next_rail,(long)rail_limit);
    return PORT_ERROR;
  }
  if (travel+shift_travel>raw_shift_budget) {
    App_Log("ROUTE2 raw chassis correction exceeds budget: need %lu mm, used %u of %u mm.\r\n",
            (unsigned long)travel,(unsigned)shift_travel,(unsigned)raw_shift_budget);
    return PORT_ERROR;
  }
  if (right_mm) {
    if (!RouteScene_IsClear() || !MotorBench_TrimShift((int16_t)right_mm)) {
      App_Print("ROUTE2 raw chassis correction blocked.\r\n");
      return PORT_ERROR;
    }
    shift_travel=(uint8_t)(shift_travel+travel);
    raw_left_mm=(int16_t)(raw_left_mm-right_mm);
    App_Log("@r2choice shift %d\r\n",(int)right_mm);
    StartSettling();
  }
  pickup_rail_target=(int16_t)next_rail;
  ++adjustments;
  phase=RAW_ADJUST;
  return PORT_WAIT;
}

static PortResult AlignRing(const MissionRequest *r) {
  if (trial>=RING_TRIAL) return AlignSingle();
  int32_t forward,left;
  int result=QrBridge_ReadRing(&forward,&left);
  if (!result) return PORT_WAIT;
  if (result<0) { App_Print("@r2choice timeout\r\nROUTE2 vision timeout.\r\n"); return PORT_ERROR; }
  App_Log("ROUTE2 %s %u fwd=%ld left=%ld um\r\n",TopTarget(r)?"top":"ring",
          r->ring,(long)forward,(long)left);
  if (left>=-LEFT_TOLERANCE_UM && left<=LEFT_TOLERANCE_UM &&
      forward>=-FORWARD_TOLERANCE_UM && forward<=FORWARD_TOLERANCE_UM) {
    if (Loading(r)) {
      /* Already facing the material. Open, descend and pick from this pose. */
      do { ++work_index; } while (RouteLibrary_Transfer(station,1U,0U,work_index,NULL)==FACE_STATION);
      phase=WORK;
      App_Print("@r2choice pickup\r\n");
      return PORT_WAIT;
    }
    if (rail_offset && !MoveVisionRail(rail_offset,0)) return PORT_ERROR;
    phase=RESTORE_TURN;
    App_Print("@r2choice aligned\r\n");
    return PORT_WAIT;
  }
  const char *reason="rail_blocked";
  if (!trial) return CorrectJointPickup(forward,left,FORWARD_TOLERANCE_UM);
  if (adjustments>=ALIGN_MAX_STEPS) { reason="retries"; goto blocked; }
  if (left < -LEFT_TOLERANCE_UM || left > LEFT_TOLERANCE_UM) {
    int32_t budget=station==ROUTE_ROUGH?rough_budget:temp_budget;
    int32_t next,travel;
    /* The lateral corridor must include the surveyed pose and sweep errors.
     * The configured budget is cumulative relative to the current coarse point. */
    shift_mm=(int16_t)(-left/1000);
    next=(int32_t)station_offset-coarse_offset+shift_mm;
    travel=shift_travel+(shift_mm<0?-shift_mm:shift_mm);
    if (rail_offset) { reason="rail_extended"; goto blocked; }
    if (next < -budget || next > budget || travel > budget) { reason="shift_budget"; goto blocked; }
    if ((int32_t)station_offset+shift_mm < -COARSE_PITCH_MM ||
        (int32_t)station_offset+shift_mm > COARSE_PITCH_MM) { reason="station_edge"; goto blocked; }
    if (!ObserveTurn(1U)) return PORT_ERROR;
    shift_travel=(uint8_t)travel;
    phase=SHIFT_TURN;
    App_Log("@r2choice shift %d\r\n",(int)shift_mm);
  } else {
    int32_t next=(int32_t)rail_offset-forward/1000;
    if (next < -(int32_t)rail_budget || next > rail_budget ||
        !ArmBench_RouteRail((int16_t)((int32_t)RAIL_NOMINAL_MM+rail_offset),
                           (int16_t)((int32_t)RAIL_NOMINAL_MM+next))) goto blocked;
    rail_offset=(int16_t)next;
    StartSettling(); phase=OBSERVE;
    App_Log("@r2choice rail %d\r\n",(int)rail_offset);
  }
  ++adjustments;
  return PORT_WAIT;
blocked:
  App_Log("@r2choice %s\r\n",reason);
  App_Print("ROUTE2 vision correction blocked by residual or travel budget.\r\n");
  return PORT_ERROR;
}
static void CommitItem(const MissionRequest *r) {
  Item item={r->batch,r->color};
  if (Loading(r)) tray[r->index]=item;
  else tray[r->index]=(Item){0};
  if (r->op==PLACE_ROUGH) rough[r->ring-1U]=item;
  if (r->op==RELOAD_ROUGH) rough[r->ring-1U]=(Item){0};
  if (r->op==PLACE_TEMP) temp[r->ring-1U]=item;
}
static PortResult Poll(const MissionRequest *r) {
  PortResult result;
  char code[16];
  switch(phase) {
    case QR:
      if (!QrBridge_GetCode(code,NULL,NULL)) return PORT_WAIT;
      if (!Mission_AcceptCode(code)) return PORT_ERROR;
      QrBridge_LockCode();
      return PORT_DONE;
    case BASE_SHIFT:
      if (!DevicesIdle()) return PORT_WAIT;
      station_offset=0; phase=BASE_ACK;
      return PORT_WAIT;
    case BASE_ACK:
      result=Confirm(r,"base");
      return result==PORT_DONE?StartLeg():result;
    case DRIVE:
      if (!DevicesIdle()) return PORT_WAIT;
      station=destination;
      if (station!=ROUTE_HOME) return PORT_DONE;
      phase=HOME_ACK;
      return PORT_WAIT;
    case HOME_ACK: return Confirm(r,"home");
    case TRIAL_ACK:
      result=Confirm(r,"ready");
      if (result!=PORT_DONE) return result;
      ready_confirmed=1U;
      phase=ALIGN;
      return PORT_WAIT;
    case FIRST_ITEM_CHECK:
      if (!DevicesIdle()) return PORT_WAIT;
      if (!ArmBench_RouteReady()) return PORT_ERROR;
      phase=ALIGN;
      return PORT_WAIT;
    case ALIGN:
      if (!DevicesIdle()) return PORT_WAIT;
      if (!Servo_Tray(r->index+1U)) return PORT_ERROR;
      StartSettling(); phase=TRAY;
      return PORT_WAIT;
    case TRAY:
      if (!Settled()) return PORT_WAIT;
      if (trial) {
        if (!Servo_Claw(0U)) return PORT_ERROR;
        StartSettling();
      }
      phase=trial>=RING_TRIAL?OBSERVE_TURN:WORK;
      return PORT_WAIT;
    case WORK: return Work(r);
    case PICKUP_OBSERVE:
      if (!Settled()) return PORT_WAIT;
      if (!(trial==RING_TRIAL?QrBridge_RequestRing(r->ring):
            QrBridge_RequestTop(trial==PICKUP_TRIAL?1U:r->ring))) return PORT_ERROR;
      phase=PICKUP_RESULT;
      return PORT_WAIT;
    case PICKUP_RESULT: return AlignSingle();
    case PICKUP_COARSE:
      if (!Settled()) return PORT_WAIT;
      phase=r->op==LOAD_RAW?WORK:trial?PICKUP_OBSERVE:OBSERVE;
      return PORT_WAIT;
    case PICKUP_RAIL:
      if (!Settled()) return PORT_WAIT;
      return ApplyPickupRail();
    case OBSERVE_TURN:
      if (!Settled()) return PORT_WAIT;
      if (!ArmBench_RouteReady() || !ObserveTurn(0U)) return PORT_ERROR;
      StartSettling(); phase=OBSERVE;
      return PORT_WAIT;
    case OBSERVE:
      if (!Settled()) return PORT_WAIT;
      if (!(TopTarget(r)?QrBridge_RequestTop(r->ring):QrBridge_RequestRing(r->ring))) return PORT_ERROR;
      phase=WAIT_RING;
      return PORT_WAIT;
    case WAIT_RING: return AlignRing(r);
    case WAIT_RAW: return AlignRaw(r);
    case RAW_ADJUST:
      if (!Settled()) return PORT_WAIT;
      if (pickup_rail_target!=rail_offset) {
        if (!MoveVisionRail(rail_offset,pickup_rail_target)) {
          App_Print("ROUTE2 raw M7 correction blocked by reference or travel.\r\n");
          return PORT_ERROR;
        }
        rail_offset=pickup_rail_target;
        App_Log("@r2choice rail %d\r\n",(int)rail_offset);
        StartSettling();
      }
      phase=WORK;
      return PORT_WAIT;
    case SHIFT_TURN:
      if (!DevicesIdle()) return PORT_WAIT;
      if (!ArmBench_CheckRouteStart()) return PORT_ERROR;
      phase=SHIFT_MOVE;
      return PORT_WAIT;
    case SHIFT_MOVE:
      if (!DevicesIdle()) return PORT_WAIT;
      if (!ArmBench_RouteReady() || !RouteScene_IsClear() ||
          !MotorBench_TrimShift(shift_mm)) return PORT_ERROR;
      station_offset+=shift_mm;
      phase=OBSERVE_TURN;
      return PORT_WAIT;
    case RESTORE_TURN:
      if (!DevicesIdle()) return PORT_WAIT;
      if (!ObserveTurn(1U)) return PORT_ERROR;
      StartSettling(); phase=WORK;
      return PORT_WAIT;
    case RETURN_CHECK:
      if (!DevicesIdle()) return PORT_WAIT;
      if (!ArmBench_RouteReady()) return PORT_ERROR;
      if (r->op==LOAD_RAW && raw_left_mm) {
        if (!RouteScene_IsClear() || !MotorBench_TrimShift(raw_left_mm)) return PORT_ERROR;
        raw_left_mm=0;
        return PORT_WAIT;
      }
      if (joint_pickup && forward_net) {
        if (!RouteScene_IsClear() || !MotorBench_TrimTravel((int16_t)-forward_net)) return PORT_ERROR;
        forward_net=0;
        return PORT_WAIT;
      }
      if (single_test) return PORT_DONE;
      phase=ITEM_ACK;
      return PORT_WAIT;
    case ITEM_ACK:
      result=Confirm(r,"done");
      if (result==PORT_DONE && !trial) CommitItem(r);
      return result;
  }
  return PORT_ERROR;
}

void Route2_Init(void) {
  active=pending=awaiting=answer=settling=trial=0U;
  no_lift=0U; route_id=2U;
  single_test=0U; single_kind="none";
  vision=raw_vision=0U;
  rough_budget=temp_budget=raw_shift_budget=SINGLE_SHIFT_LIMIT_MM;
  rail_budget=ARM_PICKUP_RAIL_LIMIT_MM; formal_forward_budget=PICKUP_TRAVEL_LIMIT_MM;
  joint_pickup=forward_budget=forward_used=0U;
  forward_net=0;
  confirmation_seq=0U;
  current_operation="idle";
  last_decision=0U;
}
void Route2_Cancel(void) { pending=active=awaiting=answer=trial=single_test=no_lift=0U; QrBridge_CancelRing(); }
uint8_t Route2_IsPending(void) { return pending; }
uint8_t Route2_IsActive(void) { return active; }
void Route2_RequestStart(uint8_t skip_lift) {
  MissionSnapshot snapshot;
  Mission_GetSnapshot(&snapshot);
  if (snapshot.home!=1U || snapshot.state!=MISSION_IDLE || !RouteScene_IsClear() ||
      !RouteLibrary_Route2Ready() || !ArmBench_IsArmed() || !Servo_ClawReady() || !Servo_OutputUs(1U) ||
      !RouteLibrary_StackDepth() || !ArmBench_HasReference()) {
    App_Print("ROUTE2 BLOCKED: check idle/home 1, obs none, arm_temp > 60 mm, geometry and arm/servo references.\r\n");
    return;
  }
  single_kind="none";
  no_lift=(uint8_t)(skip_lift!=0U); route_id=no_lift?3U:2U;
  pending=1U;
}
static void RequestTrial(const char *name,uint8_t slot) {
  MissionOp operation;
  uint8_t stack=0U;
  RouteStation target;
  if (!strcmp(name,"raw_load")) { operation=LOAD_RAW; target=ROUTE_RAW; }
  else if (!strcmp(name,"rough_place")) { operation=PLACE_ROUGH; target=ROUTE_ROUGH; }
  else if (!strcmp(name,"rough_pick") || !strcmp(name,"vision_pick")) { operation=RELOAD_ROUGH; target=ROUTE_ROUGH; }
  else if (!strcmp(name,"temp_place")) { operation=PLACE_TEMP; target=ROUTE_TEMP; }
  else if (!strcmp(name,"temp_stack")) { operation=PLACE_TEMP; target=ROUTE_TEMP; stack=1U; }
  else if (!strcmp(name,"vision_ring")) { operation=PLACE_ROUGH; target=ROUTE_ROUGH; }
  else if (!strcmp(name,"vision_stack")) { operation=PLACE_TEMP; target=ROUTE_TEMP; stack=1U; }
  else { App_Print("ROUTE2 BLOCKED: unknown arm trial.\r\n"); return; }
  if (Mission_GetState()!=MISSION_IDLE || !ArmBench_IsArmed() ||
      !Servo_ClawReady() || !Servo_OutputUs(1U) || (stack && !RouteLibrary_StackDepth()) ||
      !ArmBench_CheckRouteStart()) {
    App_Print("ROUTE2 BLOCKED: arm trial needs idle mission, arm and servo references.\r\n");
    return;
  }
  current_request=(MissionRequest){0};
  current_request.op=operation;
  current_request.index=slot-1U;
  current_request.batch=stack;
  current_request.stack=stack;
  current_operation=operation_names[operation];
  station=target;
  no_lift=0U; route_id=2U;
  trial=!strcmp(name,"vision_pick")?PICKUP_TRIAL:1U;
  if (!strcmp(name,"vision_ring")) trial=RING_TRIAL;
  if (!strcmp(name,"vision_stack")) trial=STACK_TRIAL;
  if (trial>=PICKUP_TRIAL) current_operation=name;
  joint_pickup=forward_budget=forward_used=0U;
  forward_net=0;
  single_test=0U; single_kind="none";
  pending=1U;
}
void Route2_Poll(void) {
  static const MissionPorts ports={Ready,Begin,Poll,Route2_Cancel};
  if (trial) {
    if (pending) {
      if (!DevicesIdle()) return;
      pending=0U;
      if (!ArmBench_RouteReady() || !ArmBench_IsArmed()) {
        Route2_Cancel();
        App_Print("ROUTE2 BLOCKED: arm trial reference check failed.\r\n");
        return;
      }
      active=1U;
      trial_started=HAL_GetTick();
      awaiting=answer=settling=work_index=ready_confirmed=rail_step=0U;
      rail_offset=raw_left_mm=0;
      shift_travel=0U;
      if (RelativeVisionRail()) ArmBench_ResetPickupRail();
      pickup_travel=pickup_aligned=adjustments=0U;
      current_step=WORK_NONE;
      ready_confirmed=single_test;
      phase=single_test?ALIGN:TRIAL_ACK;
      App_Print("ROUTE2 ARM TRIAL START\r\n");
      return;
    }
    uint32_t trial_limit=joint_pickup && current_request.op==LOAD_RAW?JOINT_RAW_TRIAL_LIMIT_MS:REQUEST_LIMIT_MS;
    if (HAL_GetTick()-trial_started>=trial_limit) {
      Competition_Stop("arm trial timeout",1U);
      return;
    }
    PortResult result=(trial==PICKUP_TRIAL && ((!single_test && !RouteScene_IsClear()) || MotorBench_HasFault() ||
       ArmBench_HasFault() || Servo_HasFault()))?PORT_ERROR:Poll(&current_request);
    if (result==PORT_ERROR) Competition_Stop("arm trial failed",1U);
    if (result==PORT_DONE) {
      Route2_Cancel();
      App_Print("ROUTE2 ARM TRIAL DONE\r\n");
    }
    return;
  }
  if (pending && DevicesIdle()) {
    pending=0U;
    if (!ArmBench_HasReference() || !RouteScene_IsClear() || !Mission_RegisterPorts(&ports)) {
      App_Print("ROUTE2 BLOCKED: readiness check failed.\r\n"); return;
    }
    memset(tray,0,sizeof(tray)); memset(rough,0,sizeof(rough)); memset(temp,0,sizeof(temp));
    station=ROUTE_START; station_offset=0;
    active=1U; awaiting=answer=settling=0U;
    current_operation="go_qr";
    memset(&current_request,0,sizeof(current_request));
    phase=DRIVE;
    if (!Mission_StartCommissioning(COMMISSIONING_LIMIT_MS,REQUEST_LIMIT_MS)) { Route2_Cancel(); return; }
    App_Print("ROUTE2 COMMISSIONING START\r\n");
    App_Log("@r2mode %u\r\n",route_id);
    if(no_lift) App_Print("ROUTE3 START: M6 read-only at recorded top; transfers/counts SIMULATED.\r\n");
  }
  if (active && Mission_GetState()==MISSION_DONE) {
    Route2_Cancel();
    Mission_ShowStatus();
  }
}
void Route2_Report(void) {
  MissionSnapshot snapshot;
  Mission_GetSnapshot(&snapshot);
  App_Log("@r2mode %u\r\n",route_id);
  App_Log("@r2trial %u\r\n",trial!=0U);
  App_Log("@r2single %s %u %u %u %u %u %u %u\r\n",single_kind,
    (unsigned)current_request.index+1U,(unsigned)current_request.ring,pickup_budget,pickup_travel,
    (unsigned)(!strcmp(single_kind,"raw")?current_request.color:0U),
    (unsigned)forward_budget,(unsigned)forward_used);
  App_Log("@r2state %u %u %u %u %s %lu %u %u %u %u %u %lu %u %u %s %u\r\n",
    active,pending,(unsigned)snapshot.state,snapshot.home,active?current_operation:"idle",
    (unsigned long)RouteLibrary_StackDepth(),RouteScene_IsClear(),vision,rough_budget,temp_budget,rail_budget,
    (unsigned long)(awaiting && !answer?confirmation_seq:0U),snapshot.grabs,snapshot.places,
    snapshot.task_code[0]?snapshot.task_code:"unset",
    (unsigned)(RouteLibrary_Route2Ready() | (Servo_ClawReady()<<1)));
}
void Route2_ReportDecision(uint8_t force) {
  static const char *const phases[]={"qr","base_shift","base_ack","drive","home_ack","align","tray",
    "work","return_check","item_ack","observe_turn","observe","wait_ring","shift_turn","shift_move",
    "restore_turn","trial_ack","wait_raw","first_item_check",
    "pickup_observe","pickup_result","pickup_rail","raw_adjust","pickup_coarse"};
  const MissionRequest *r=&current_request;
  uint32_t key=active?3U+r->op*65536U+r->batch*32768U+r->index*8192U+phase*256U+work_index*16U+(phase==WORK?current_step:0U)
                     :pending?2U:1U;
  if (!force && (key==last_decision || (!last_decision && key==1U))) return;
  last_decision=key;
  if (pending) App_Print("@r2decision prepare\r\n");
  else if (!active) App_Print("@r2decision idle\r\n");
  else App_Log("@r2decision %s %s %u %u %u %u %u %u %s\r\n",
    current_operation,phases[phase],(unsigned)r->batch+1U,(unsigned)r->index+1U,
    (unsigned)r->color,(unsigned)(r->op==LOAD_RAW?0U:r->ring),r->stack?2U:1U,
    phase==WORK?(unsigned)work_index:0U,
    transfer_names[phase==WORK?current_step:WORK_NONE]);
}
void Route2_Command(const char *command,uint8_t allow_config) {
  char line[64],*op,*arg,*result;
  uint32_t value;
  if (!strcmp(command,"status")) {
    uint32_t stack_depth=RouteLibrary_StackDepth();
    Route2_Report();
    Route2_ReportDecision(1U);
    App_Log("route2 active=%u pending=%u stack=%lu.%03lu mm obs-clear=%u\r\n",
      active,pending,(unsigned long)(stack_depth/1000U),(unsigned long)(stack_depth%1000U),RouteScene_IsClear());
    if (awaiting && !answer) {
      App_Log("route2 waiting %s id=%lu\r\n",confirmation_kind,(unsigned long)confirmation_seq);
      ReportConfirmation(&confirmation_request);
    }
    App_Log("route2 vision=%u rawvision=%u budgets D/E/M7=%u/%u/%u mm RAW-left=%u mm\r\n",
      vision,raw_vision,rough_budget,temp_budget,rail_budget,raw_shift_budget);
    App_Log("route2 formal fore/aft cumulative=%u mm; work-zone boundary enforced.\r\n",formal_forward_budget);
    return;
  }
  if (strlen(command)>=sizeof(line)) goto invalid;
  strcpy(line,command); op=strtok(line," "); arg=strtok(NULL," "); result=strtok(NULL," ");
  if (op && arg && !strcmp(op,"single") && !strcmp(arg,"raw")) {
    char *color_text=strtok(NULL," "),*side_text=strtok(NULL," "),*forward_text=strtok(NULL," ");
    uint32_t slot,color,side=0U,fore=0U;
    if (!allow_config || !result || !color_text || strtok(NULL," ") ||
        !App_ParseU32(result,1U,3U,&slot) || !App_ParseU32(color_text,1U,6U,&color) ||
        (side_text && (!forward_text || !RouteScene_IsClear() ||
        !App_ParseU32(side_text,0U,SINGLE_SHIFT_LIMIT_MM,&side) ||
        !App_ParseU32(forward_text,0U,PICKUP_TRAVEL_LIMIT_MM,&fore)))) goto invalid;
    RequestTrial("raw_load",(uint8_t)slot);
    if (!pending) return;
    current_request.color=(uint8_t)color;
    single_test=1U;
    single_kind="raw";
    joint_pickup=side_text!=NULL;
    pickup_budget=(uint8_t)side; forward_budget=(uint8_t)fore;
    pickup_travel=0U;
    return;
  }
  if (op && !strcmp(op,"single")) {
    char *first=strtok(NULL," "),*second=strtok(NULL," "),*third=strtok(NULL," ");
    uint32_t slot,ring=1U,budget,fore=0U;
    uint8_t ground=(uint8_t)(arg && !strcmp(arg,"ground"));
    const char *side_text=ground?first:second;
    const char *forward_text=ground?second:third;
    if (!allow_config || !arg || !result || !side_text ||
        (!ground && strcmp(arg,"ring") && strcmp(arg,"stack")) ||
        !App_ParseU32(result,1U,3U,&slot) || (ground && third) || strtok(NULL," ") ||
        (!ground && !App_ParseU32(first,1U,3U,&ring)) ||
        (forward_text && (!RouteScene_IsClear() ||
        !App_ParseU32(forward_text,0U,PICKUP_TRAVEL_LIMIT_MM,&fore))) ||
        !App_ParseU32(side_text,0U,SINGLE_SHIFT_LIMIT_MM,&budget)) goto invalid;
    RequestTrial(ground?"vision_pick":!strcmp(arg,"ring")?"vision_ring":"vision_stack",(uint8_t)slot);
    if (!pending) return;
    current_request.ring=(uint8_t)ring;
    single_test=1U;
    single_kind=ground?"ground":trial==RING_TRIAL?"ring":"stack";
    joint_pickup=forward_text!=NULL;
    forward_budget=(uint8_t)fore;
    pickup_budget=(uint8_t)budget; pickup_travel=0U;
    return;
  }
  if (allow_config && op && arg && !strcmp(op,"limits")) {
    char *rail=strtok(NULL," ");
    char *fore=strtok(NULL," ");
    uint32_t d,e,m,f=PICKUP_TRAVEL_LIMIT_MM;
    if (!result || !rail || strtok(NULL," ") || !App_ParseU32(arg,0U,SINGLE_SHIFT_LIMIT_MM,&d) ||
        !App_ParseU32(result,0U,SINGLE_SHIFT_LIMIT_MM,&e) || !App_ParseU32(rail,0U,ARM_PICKUP_RAIL_LIMIT_MM,&m) ||
        (fore && !App_ParseU32(fore,0U,PICKUP_TRAVEL_LIMIT_MM,&f))) goto invalid;
    rough_budget=(uint8_t)d; temp_budget=(uint8_t)e; rail_budget=(uint8_t)m;
    formal_forward_budget=(uint8_t)f;
    App_Print("ROUTE2 surveyed budgets set in RAM.\r\n"); return;
  }
  if (allow_config && op && arg && !strcmp(op,"rawlimit") && !result &&
      App_ParseU32(arg,0U,RAW_SHIFT_LIMIT_MM,&value)) {
    raw_shift_budget=(uint8_t)value;
    App_Log("ROUTE2 raw chassis budget=%u mm\r\n",(unsigned)raw_shift_budget);
    return;
  }
  if (!op || !arg) goto invalid;
  if (!strcmp(op,"pickup") && result) {
    const char *forward_text=strtok(NULL," ");
    uint32_t budget,fore=0U;
    if (!allow_config || !RouteScene_IsClear() || MotorBench_HasFault() ||
        !App_ParseU32(arg,1U,3U,&value) ||
        !App_ParseU32(result,0U,SINGLE_SHIFT_LIMIT_MM,&budget) || strtok(NULL," ") ||
        (forward_text && !App_ParseU32(forward_text,0U,PICKUP_TRAVEL_LIMIT_MM,&fore))) goto invalid;
    RequestTrial("vision_pick",(uint8_t)value);
    if (!pending) return;
    pickup_budget=(uint8_t)budget;
    forward_budget=(uint8_t)fore;
    joint_pickup=forward_text!=NULL;
    return;
  }
  if (strtok(NULL," ")) goto invalid;
  if (allow_config && !result && (!strcmp(op,"vision") || !strcmp(op,"rawvision")) &&
      (!strcmp(arg,"on") || !strcmp(arg,"off"))) {
    if (!strcmp(op,"vision")) vision=(uint8_t)!strcmp(arg,"on");
    else raw_vision=(uint8_t)!strcmp(arg,"on");
    App_Log("ROUTE2 vision=%u rawvision=%u\r\n",vision,raw_vision); return;
  }
  if (!strcmp(op,"trial") && result) {
    if (!allow_config || !App_ParseU32(result,1U,3U,&value)) {
      App_Print("ROUTE2 BLOCKED: arm trial busy/fault or invalid slot.\r\n");
      return;
    }
    if (!strcmp(arg,"vision_pick") || !strcmp(arg,"vision_ring") || !strcmp(arg,"vision_stack")) goto invalid;
    RequestTrial(arg,(uint8_t)value);
    return;
  }
  if (!strcmp(op,"ack") && result && active && awaiting && !answer &&
      HAL_GetTick()-confirmation_at<CONFIRM_TIMEOUT_MS &&
      App_ParseU32(arg,1U,UINT32_MAX,&value) && value==confirmation_seq &&
      (!strcmp(result,"ok") || !strcmp(result,"fail"))) {
    answer=!strcmp(result,"ok")?1U:2U;
    App_Log("ROUTE2 ACK %lu %s\r\n",(unsigned long)value,result);
    return;
  }
invalid:
  App_Print("ROUTE2 single ground SLOT SIDE [FWD] | single raw SLOT COLOR [SIDE FWD] | single ring|stack SLOT RING SIDE [FWD]\r\n");
  App_Print("or status/ack ID ok|fail/trial NAME SLOT/pickup SLOT SIDE [FWD]/vision on|off/rawvision on|off/limits D E M7/rawlimit MM.\r\n");
}
