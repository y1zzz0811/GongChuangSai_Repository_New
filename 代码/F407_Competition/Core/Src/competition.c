#include "competition.h"
#include "app_console.h"
#include "motor_bench.h"
#include "arm_bench.h"
#include "mission.h"
#include "qr_bridge.h"
#include "servo.h"
#include "route_library.h"
#include "route2.h"
#include "route_scene.h"
#include <string.h>
static uint8_t system_fault,propagated,handling,pending_route;
static uint32_t handling_tick,live_tick,route2_tick;
static UART_HandleTypeDef *chassis_uart,*arm_uart;
static uint8_t Idle(void) {
  return !pending_route && !Route2_IsPending() && !Route2_IsActive() &&
    !MotorBench_IsBusy() && !ArmBench_IsBusy() && !Mission_IsRunning();
}
static uint8_t HasFault(void) {
  return (uint8_t)(system_fault || MotorBench_HasFault() || ArmBench_HasFault() ||
                   Servo_HasFault() || Mission_GetState()==MISSION_FAULT);
}
static void ReportState(void) {
  uint8_t path,step,count;
  const char *phase=MotorBench_LivePhase(&path,&step,&count);
  App_Log("@state %u %u %s %s %u %u %u %u %u %u %u %u\r\n",
    (unsigned)ArmBench_IsArmed(),(unsigned)HasFault(),phase,ArmBench_LivePhase(),
    (unsigned)path,(unsigned)step,(unsigned)count,(unsigned)MotorBench_WorkGroup(),(unsigned)handling,
    (unsigned)Servo_OutputUs(0U),(unsigned)Servo_OutputUs(1U),(unsigned)Servo_IsBusy());
  if (!AppConsole_StopPending()) ArmBench_ReportReadiness();
}
void Competition_Init(UART_HandleTypeDef *pc,UART_HandleTypeDef *chassis,UART_HandleTypeDef *arm) {
  system_fault=propagated=handling=pending_route=0U; live_tick=route2_tick=HAL_GetTick(); chassis_uart=chassis; arm_uart=arm;
  AppConsole_Init(pc); MotorBench_Init(pc,chassis); ArmBench_Init(pc,arm); Mission_Init();
  RouteScene_Init(); Route2_Init();
  App_Print("F407 Competition C6.57. No auto motion.\r\n> ");
}
void Competition_Stop(const char *reason,uint8_t fault) {
  uint8_t f[]={0U,0xFEU,0x98U,0U,0x6BU};
  handling=pending_route=0U; Servo_Stop();
  /* Stop BOTH physical buses before logging or service cancellation. */
  (void)HAL_UART_Transmit(chassis_uart,f,sizeof(f),10U);
  (void)HAL_UART_Transmit(arm_uart,f,sizeof(f),10U);
  if (fault) system_fault=1U;
  propagated=1U; AppConsole_ClearInput(); Mission_Stop(fault); Route2_Cancel();
  MotorBench_Stop(reason,fault); ArmBench_Stop(reason,fault);
}
uint8_t Competition_RequestStart(void) {
  if (!Idle() || Servo_IsBusy() || system_fault || MotorBench_HasFault() || ArmBench_HasFault() || Servo_HasFault()) {
    App_Print("START BLOCKED: busy/fault.\r\n"); return 0U;
  }
  return Mission_Start();
}
/* Station groups own the arm until all accepted moves have completed.
 * Future vision alignment belongs before dispatching external descents. */
static void HandlingPoll(void) {
  RouteWorkStep action; uint8_t group=MotorBench_WorkGroup(); const char *reason="route work rejected";
  if(!group) return;
  if(!handling) {
    if(!Servo_ClawReady()) { reason="route requires S1 reference"; goto fail; }
    handling=1U;
  } else {
    if(HAL_GetTick()-handling_tick>120000U) { reason="route work timeout"; goto fail; }
    if(ArmBench_IsBusy() || Servo_IsBusy()) return;
    ++handling;
  }
  if(handling>RouteLibrary_WorkCount(group)) {
    handling=0U; MotorBench_CompleteWork(); return;
  }
  if(!RouteLibrary_Work(group,handling-1U,&action)) goto fail;
  if(action.claw && !Servo_Claw(action.claw==2U)) goto fail;
  if(action.axis!=255U && !ArmBench_RouteAction(action.axis,action.negative,action.amount,action.rpm)) goto fail;
  App_Log("WORK group=%u step=%u axis=%u negative=%u amount=%lu rpm=%u claw=%u\r\n",
    (unsigned)group,(unsigned)handling,(unsigned)action.axis,(unsigned)action.negative,
    (unsigned long)action.amount,(unsigned)action.rpm,(unsigned)action.claw);
  handling_tick=HAL_GetTick(); return;
fail:
  App_Log("route work group=%u step=%u\r\n",group,handling);
  Competition_Stop(reason,1U);
}
static void StartPath(uint8_t id) {
  if (id==2U || id==3U) { Route2_RequestStart((uint8_t)(id==3U)); return; }
  if (id!=1U) {
    if (!MotorBench_StartPath(id)) App_Print("Invalid path/config.\r\n");
    return;
  }
  if (MotorBench_IsBusy() || !RouteLibrary_Validate(id)) {
    App_Print("START BLOCKED: busy or invalid path/config.\r\n"); return;
  }
  if (!ArmBench_CheckRouteStart()) {
    App_Print("START BLOCKED: arm ref set and M7 forward 65mm required.\r\n"); return;
  }
  pending_route=1U;
}
static void ProcessCommand(const char *cmd) {
  uint8_t arm; const char *sub;
  if(!strcmp(cmd,"telemetry on") || !strcmp(cmd,"telemetry off")) {
    AppConsole_SetTelemetry((uint8_t)!strcmp(cmd,"telemetry on"));
    App_Log("Telemetry %s; status queries remain available.\r\n",AppConsole_Telemetry()?"on":"off");
    return;
  }
  if (!strcmp(cmd,"stop") || !strcmp(cmd,"stopall")) { Competition_Stop("operator stop",0U); return; }
  if (!strcmp(cmd,"help")) {
    App_Print("chassis/arm/servo CMD | route [show/set/reset/export/start/next/status/stop]\r\nroute2 status/ack/trial/pickup/vision/limits | obs none/status/set\r\nstart/status/stopall/!/clearfault\r\nmission/qr: see manual\r\n"); return;
  }
  if (!strcmp(cmd,"status")) { ReportState(); Mission_ShowStatus(); MotorBench_Command("status"); ArmBench_Command("status"); Servo_ShowStatus(); return; }
  if (!strncmp(cmd,"obs ",4U)) {
    if (RouteScene_Command(cmd+4U,Idle()) && (Route2_IsPending() || Route2_IsActive()))
      Competition_Stop("route2 obstacle; no commissioned detour",1U);
    return;
  }
  if (!strncmp(cmd,"route2 ",7U)) {
    Route2_Command(cmd+7U,Idle() && !Servo_IsBusy() && !HasFault());
    return;
  }
  if (!strncmp(cmd,"servo ",6U)) {
    sub=cmd+6U;
    if (!strcmp(sub,"stop")) { Competition_Stop("operator stop",0U); return; }
    if (!strcmp(sub,"status") || !strcmp(sub,"help") || !strcmp(sub,"preset export")) { Servo_Command(sub); return; }
    if (!Idle() || system_fault || MotorBench_HasFault() || ArmBench_HasFault() || Servo_HasFault()) {
      App_Print("Servo change blocked: busy/fault.\r\n"); return;
    }
    Servo_Command(sub); return;
  }
  if(!strcmp(cmd,"route") || !strncmp(cmd,"route ",6U)) {
    const char *r=cmd[5]?cmd+6U:""; uint32_t id;
    if(!strcmp(r,"stop")) { Competition_Stop("operator stop",0U); return; }
    if(!strcmp(r,"status")) { MotorBench_Command("status"); App_Log("work step=%u\r\n",handling); Route2_Command("status",0U); return; }
    if(!strcmp(r,"show 2") || !strcmp(r,"show 3")) {
      App_Print("route 2/3: A -> B -> C -> D -> E -> C -> D -> E -> A; route3 skips M6 motion only.\r\n");
      Route2_Command("status",0U); return;
    }
    if(!r[0] || !strcmp(r,"show") || !strncmp(r,"show ",5U) || !strcmp(r,"export")) {
      RouteLibrary_Command(r);
      if (!r[0]) App_Print("route 2 commissioning; route 3 no-lift demonstration (two batches)\r\n");
      return;
    }
    if(!strcmp(r,"next")) {
      if(!system_fault && !MotorBench_HasFault() && !ArmBench_HasFault() && !Servo_HasFault() &&
         !Mission_IsRunning() && !Route2_IsPending() && !Servo_IsBusy() && !ArmBench_IsBusy() && !MotorBench_WorkGroup()) MotorBench_Command("next");
      return;
    }
    if(!Idle() || Servo_IsBusy() || system_fault || MotorBench_HasFault() || ArmBench_HasFault() || Servo_HasFault()) {
      App_Print("Route change blocked: busy/fault.\r\n"); return;
    }
    if(!strncmp(r,"start ",6U)) {
      if(!App_ParseU32(r+6U,1U,255U,&id)) App_Print("Invalid path/config.\r\n");
      else StartPath((uint8_t)id);
    } else RouteLibrary_Command(r);
    return;
  }
  if (!strcmp(cmd,"mission status")) { Mission_ShowStatus(); return; }
  if (!strcmp(cmd,"qr status")) { QrBridge_ShowStatus(); return; }
  if (!strcmp(cmd,"qr clear") || !strcmp(cmd,"qr resend")) {
    if (!Idle()) { App_Print("Busy: QR settings locked.\r\n"); return; }
    if (!strcmp(cmd,"qr clear")) QrBridge_Clear(); else QrBridge_Resend();
    return;
  }
  if (!strcmp(cmd,"clearfault")) {
    if (!Idle()) { App_Print("Busy: wait for stop ACKs.\r\n"); return; }
    MotorBench_Command("clearfault"); ArmBench_Command("clearfault");
    if (!MotorBench_HasFault() && !ArmBench_HasFault() && !Servo_HasFault()) {
      system_fault=propagated=0U; Mission_Reset(); App_Print("Cleared; arm locked; no resume.\r\n");
    } else if (Servo_HasFault()) App_Print("Servo init fault: inspect and reset.\r\n");
    return;
  }
  if (!strcmp(cmd,"start") || !strcmp(cmd,"\xE5\x90\xAF\xE5\x8A\xA8")) { (void)Competition_RequestStart(); return; }
  if (!strncmp(cmd,"mission ",8U)) {
    if (!Idle() || Servo_IsBusy() || system_fault || MotorBench_HasFault() || ArmBench_HasFault() || Servo_HasFault()) { App_Print("Mission busy/fault.\r\n"); return; }
    if (!strcmp(cmd,"mission reset")) Mission_Reset();
    else if (!strcmp(cmd,"mission home 1")) (void)Mission_SetHome(1U);
    else if (!strcmp(cmd,"mission home 2")) (void)Mission_SetHome(2U);
    else App_Print("Use mission status/reset/home 1/2.\r\n");
    return;
  }
  if (!strncmp(cmd,"route1 ",7U)) {
    arm=0U; sub=cmd+7U;
    if (!strcmp(sub,"start")) sub="route1";
    else if (strcmp(sub,"next") && strcmp(sub,"status") && strcmp(sub,"stop")) {
      App_Print("route1 start/next/status/stop\r\n"); return;
    }
  } else {
    arm=(uint8_t)(!strncmp(cmd,"arm ",4U));
    if (!arm && strncmp(cmd,"chassis ",8U)) { App_Print("Unknown command; help\r\n"); return; }
    sub=cmd+(arm?4U:8U);
  }
  if (!strcmp(sub,"stop") || !strcmp(sub,"stopall") || (arm && !strcmp(sub,"off"))) {
    Competition_Stop("operator stop",0U); return;
  }
  if (!strcmp(sub,"status") || !strcmp(sub,"help") || (arm && !strcmp(sub,"ref status"))) {
    if (arm) ArmBench_Command(sub); else MotorBench_Command(sub); return;
  }
  if (!arm && !strcmp(sub,"next") && MotorBench_WorkGroup()) return;
  if (Servo_IsBusy()) { App_Print("Servo ramp busy; wait or stopall.\r\n"); return; }
  if (system_fault || MotorBench_HasFault() || ArmBench_HasFault() || Servo_HasFault() ||
      Mission_IsRunning() || Route2_IsPending() || Route2_IsActive()) {
    App_Print("Fault/mission: status/stopall/clearfault.\r\n"); return;
  }
  if ((arm && MotorBench_IsBusy()) || (!arm && ArmBench_IsBusy())) {
    App_Print("Other subsystem busy.\r\n"); return;
  }
  if (!arm && (!strcmp(sub,"start") || !strcmp(sub,"\xE5\x90\xAF\xE5\x8A\xA8"))) {
    App_Print("chassis square MM; start=mission\r\n"); return;
  }
  if (arm) ArmBench_Command(sub);
  else if (!strcmp(sub,"route1")) StartPath(1U);
  else MotorBench_Command(sub);
}
void Competition_Command(const char *input) {
  /* Normalize BEFORE stop routing, interlocks or module parsing. */
  static char cmd[160];
  size_t n=0U; uint8_t space=0U;
  if (!input || strlen(input)>=sizeof(cmd)) { App_Print("Command too long.\r\n"); return; }
  while (*input) {
    char c=*input++;
    if (c==' ' || c=='\t') { space=(uint8_t)(n!=0U); continue; }
    if (space) cmd[n++]=' ';
    space=0U; cmd[n++]=(c>='A' && c<='Z')?(char)(c-'A'+'a'):c;
  }
  cmd[n]='\0';
  if (cmd[0]) ProcessCommand(cmd);
}
void Competition_Poll(void) {
  AppConsole_Poll(); if (AppConsole_StopPending()) return;
  MotorBench_Poll(); if (AppConsole_StopPending()) return;
  ArmBench_Poll(); if (AppConsole_StopPending()) return;
  QrBridge_Poll(); if (AppConsole_StopPending()) return;
  if (HasFault()) {
    if (!propagated) Competition_Stop("subsystem fault propagated",1U);
  } else {
    propagated=0U;
    if (pending_route && !ArmBench_IsBusy()) {
      pending_route=0U;
      if (!ArmBench_RouteReady() || !MotorBench_StartPath(1U)) App_Print("START BLOCKED: arm reference check failed.\r\n");
    }
    HandlingPoll();
    if (system_fault) return;
    Route2_Poll();
    if (system_fault) return;
    Mission_Poll();
    if (Mission_GetState()==MISSION_FAULT) Competition_Stop("mission failed/timeout",1U);
  }
  if (!AppConsole_StopPending()) Servo_Poll();
  if (!AppConsole_StopPending()) Route2_ReportDecision(0U);
  if (AppConsole_Telemetry() && !AppConsole_StopPending() && HAL_GetTick()-live_tick>=300U) {
    live_tick=HAL_GetTick(); ReportState();
  }
  if (AppConsole_Telemetry() && !AppConsole_StopPending() && HAL_GetTick()-route2_tick>=750U) {
    route2_tick=HAL_GetTick(); Route2_Report();
  }
}
