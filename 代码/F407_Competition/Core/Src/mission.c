#include "mission.h"
#include "app_console.h"
#include "stm32f4xx_hal.h"
#include <string.h>
static MissionPorts ports;
static MissionCode code;
static MissionRequest request;
static MissionState state;
static uint8_t stage,begun,has_code,home,grabs,places;
static uint32_t started,step_started,elapsed_ms;
static uint32_t total_limit,request_limit;
static const MissionOp stages[]={GO_QR,READ_QR,GO_RAW,LOAD_RAW,GO_ROUGH,PLACE_ROUGH,RELOAD_ROUGH,GO_TEMP,PLACE_TEMP,GO_RAW,LOAD_RAW,GO_ROUGH,PLACE_ROUGH,RELOAD_ROUGH,GO_TEMP,PLACE_TEMP,GO_HOME};
static uint8_t ItemOp(MissionOp op) { return (uint8_t)(op==LOAD_RAW || op==PLACE_ROUGH || op==RELOAD_ROUGH || op==PLACE_TEMP); }
void Mission_Init(void) { memset(&ports,0,sizeof(ports)); state=MISSION_IDLE; home=1U; Mission_Reset(); }
uint8_t Mission_RegisterPorts(const MissionPorts *p) {
  if (state==MISSION_RUNNING) return 0U;
  if (p) ports=*p; else memset(&ports,0,sizeof(ports));
  return 1U;
}
uint8_t Mission_ParseCode(const char *s,MissionCode *out) {
  MissionCode parsed; uint8_t batch,j,mask[2]={0U,0U};
  if (!s || !out || strlen(s)!=15U || s[3]!='+' || s[7]!='+' || s[11]!='+') return 0U;
  memset(&parsed,0,sizeof(parsed));
  for (batch=0U;batch<2U;++batch) {
    uint8_t rings=0U;
    for (j=0U;j<3U;++j) {
      char c=s[batch*8U+j],r=s[batch*8U+4U+j];
      uint8_t bit;
      if (c<'1' || c>'6' || r<'1' || r>'3') return 0U;
      parsed.colors[batch][j]=(uint8_t)(c-'0'); parsed.rings[batch][j]=(uint8_t)(r-'0');
      bit=(uint8_t)(1U<<(c-'1')); if (mask[batch]&bit) return 0U; mask[batch]|=bit;
      bit=(uint8_t)(1U<<(r-'1')); if (rings&bit) return 0U; rings|=bit;
      if (!batch) parsed.temp_ring[(unsigned)(c-'0')]=(uint8_t)(r-'0');
    }
  }
  if (mask[0]!=mask[1]) return 0U;
  memcpy(parsed.text,s,16U); *out=parsed; return 1U;
}
uint8_t Mission_AcceptCode(const char *s) {
  if (state!=MISSION_RUNNING || request.op!=READ_QR || has_code) return 0U;
  if (!Mission_ParseCode(s,&code)) return 0U;
  has_code=1U; return 1U;
}
static void MakeRequest(void) {
  request.op=stages[stage]; request.batch=(uint8_t)(stage>=9U); request.home=home;
  request.stack=(uint8_t)(request.batch && request.op==PLACE_TEMP);
  request.color=request.ring=0U;
  if (ItemOp(request.op)) {
    request.color=code.colors[request.batch][request.index];
    request.ring=(request.op==PLACE_TEMP)?code.temp_ring[request.color]:code.rings[request.batch][request.index];
  }
}
static uint8_t Start(uint32_t total_ms,uint32_t request_ms) {
  uint32_t ready=ports.ready?ports.ready():0U;
  if (state!=MISSION_IDLE) { App_Print("Mission requires reset while idle.\r\n"); return 0U; }
  if (ready!=SERVICE_ALL || !ports.begin || !ports.poll || !ports.cancel) {
    App_Log("START BLOCKED: unconnected/unready services mask=%02X, need=3F. No motion.\r\n",(unsigned)ready); return 0U;
  }
  memset(&code,0,sizeof(code)); memset(&request,0,sizeof(request));
  total_limit=total_ms; request_limit=request_ms;
  stage=begun=has_code=grabs=places=0U; elapsed_ms=0U; state=MISSION_RUNNING; started=step_started=HAL_GetTick(); MakeRequest();
  return 1U;
}
uint8_t Mission_Start(void) { return Start(180000U,0U); }
uint8_t Mission_StartCommissioning(uint32_t total_ms,uint32_t request_ms) {
  if (!request_ms || request_ms>total_ms) return 0U;
  return Start(total_ms,request_ms);
}
void Mission_Poll(void) {
  PortResult result; uint32_t now=HAL_GetTick();
  if (state!=MISSION_RUNNING) return;
  if (now-started>=total_limit ||
      now-step_started>=(request_limit?request_limit:((request.op==LOAD_RAW)?23000U:14000U)) ||
      !ports.ready || ports.ready()!=SERVICE_ALL) { Mission_Stop(1U); return; }
  if (!begun) { begun=1U; result=ports.begin(&request); }
  else result=ports.poll(&request);
  if (result==PORT_ERROR) { Mission_Stop(1U); return; }
  if (result!=PORT_DONE) return;
  if (request.op==READ_QR && !has_code) { Mission_Stop(1U); return; }
  if (request.op==LOAD_RAW || request.op==RELOAD_ROUGH) ++grabs;
  if (request.op==PLACE_ROUGH || request.op==PLACE_TEMP) ++places;
  if (ItemOp(request.op) && ++request.index<3U) {
    begun=0U;
    if (request_limit) step_started=now;
    MakeRequest(); return;
  }
  request.index=0U; begun=0U; step_started=now;
  if (++stage>=sizeof(stages)/sizeof(stages[0])) { elapsed_ms=now-started; state=MISSION_DONE; return; }
  MakeRequest();
}
void Mission_Stop(uint8_t fault) {
  if (state==MISSION_RUNNING) { elapsed_ms=HAL_GetTick()-started; if (ports.cancel) ports.cancel(); }
  state=fault?MISSION_FAULT:MISSION_STOPPED; begun=0U;
}
void Mission_Reset(void) {
  if (state==MISSION_RUNNING) return;
  state=MISSION_IDLE; stage=begun=has_code=grabs=places=0U; elapsed_ms=0U; memset(&code,0,sizeof(code)); memset(&request,0,sizeof(request));
}
uint8_t Mission_SetHome(uint8_t h) { if (state==MISSION_RUNNING || h<1U || h>2U) return 0U; home=h; return 1U; }
MissionState Mission_GetState(void) { return state; }
uint8_t Mission_IsRunning(void) { return (uint8_t)(state==MISSION_RUNNING); }
void Mission_GetSnapshot(MissionSnapshot *out) {
  if (!out) return;
  memset(out,0,sizeof(*out)); out->state=state; out->home=home;
  out->grabs=grabs; out->places=places;
  out->elapsed_ms=(state==MISSION_RUNNING)?HAL_GetTick()-started:elapsed_ms;
  if (has_code) memcpy(out->task_code,code.text,sizeof(out->task_code));
}
void Mission_ShowStatus(void) {
  App_Log("mission=%u home=%u stage=%u item=%u code=%s grabs=%u places=%u ready=%02X\r\n",
    (unsigned)state,(unsigned)home,(unsigned)stage,(unsigned)request.index,has_code?code.text:"unset",
    (unsigned)grabs,(unsigned)places,(unsigned)(ports.ready?ports.ready():0U));
}
