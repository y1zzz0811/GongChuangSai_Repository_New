#include "servo.h"
#include "servo_config.h"
#include "app_console.h"
#include <string.h>
typedef struct { uint16_t current,target,speed; uint32_t fraction; } Channel;
static Channel channels[2];
static const Channel defaults[2]={
  {0,0,SERVO_S1_SPEED_DPS,0}, {0,0,SERVO_S2_SPEED_DPS,0}
};
static const uint16_t tray_defaults[3]={SERVO_TRAY_1_US,SERVO_TRAY_2_US,SERVO_TRAY_3_US};
static uint16_t tray_positions[3];
static TIM_HandleTypeDef *pwm;
static uint16_t claw_open,claw_close;
static uint32_t last_tick;
static uint8_t ready,fault;
static uint8_t Valid(uint32_t v) { return (uint8_t)(v>=SERVO_MIN_US && v<=SERVO_MAX_US); }
/* State/API destinations use presets; raw debugging and ramps use bounds. */
static uint8_t Allowed(unsigned i,uint32_t us) {
  return (uint8_t)(Valid(us) && (i?1U:
    (us>=claw_open && us<=SERVO_CLAW_HARD_MAX_US)));
}
static uint8_t Destination(unsigned i,uint32_t us) {
  return (uint8_t)(Allowed(i,us) && (i?
    (us==tray_positions[0] || us==tray_positions[1] || us==tray_positions[2]):
    (us==claw_open || us==claw_close)));
}
static void Write(unsigned i,uint16_t us) {
  /* Last barrier for a future internal caller/math error. User-invalid inputs
   * are rejected earlier without changing either output or target. */
  if (us && !Allowed(i,us)) { fault=1U; us=0U; channels[i].target=0U; channels[i].fraction=0U; }
  channels[i].current=us;
  if (pwm) __HAL_TIM_SET_COMPARE(pwm,i==0U?TIM_CHANNEL_1:TIM_CHANNEL_2,us);
}
static void Target(unsigned i,uint16_t us) {
  Channel *c=&channels[i];
  if (!Allowed(i,us)) { App_Print("Servo pulse out of range.\r\n"); return; }
  if (!c->current) { App_Print("Servo start unknown; setup unloaded.\r\n"); return; }
  c->target=us; c->fraction=0U;
}
void Servo_Stop(void) {
  unsigned i;
  for (i=0U;i<2U;++i) { Write(i,0U); channels[i].target=0U; channels[i].fraction=0U; }
  last_tick=HAL_GetTick();
}
void Servo_Init(TIM_HandleTypeDef *timer) {
  unsigned i;
  pwm=timer; ready=0U; fault=1U;
  memcpy(channels,defaults,sizeof(channels)); memcpy(tray_positions,tray_defaults,sizeof(tray_positions)); claw_open=SERVO_CLAW_OPEN_US; claw_close=SERVO_CLAW_CLOSE_US;
  Servo_Stop();
  for (i=0U;i<2U;++i) {
    Channel *c=&channels[i];
    if (!c->speed || c->speed>360U) return;
  }
  if (!Valid(claw_open) || claw_close<=claw_open || !Allowed(0U,claw_close) ||
      !timer || timer->Init.Prescaler!=83U || timer->Init.Period!=19999U ||
      HAL_RCC_GetPCLK1Freq()!=42000000U) return;
  if (HAL_TIM_GenerateEvent(timer,TIM_EVENTSOURCE_UPDATE)!=HAL_OK ||
      HAL_TIM_PWM_Start(timer,TIM_CHANNEL_1)!=HAL_OK ||
      HAL_TIM_PWM_Start(timer,TIM_CHANNEL_2)!=HAL_OK) return;
  ready=1U; fault=0U;
}
uint8_t Servo_HasFault(void) { return fault; }
uint16_t Servo_OutputUs(uint8_t channel) { return channels[channel].current; }
uint8_t Servo_IsBusy(void) {
  return (uint8_t)(channels[0].current!=channels[0].target || channels[1].current!=channels[1].target);
}
void Servo_Poll(void) {
  unsigned i; uint32_t now=HAL_GetTick();
  if (!ready || fault || now-last_tick<20U) return;
  /* Drop missed frames: a delayed main loop must never jump to catch up. */
  last_tick=now;
  for (i=0U;i<2U;++i) {
    Channel *c=&channels[i]; uint32_t step,denom,remaining;
    if (c->current==c->target) continue;
    denom=SERVO_NOMINAL_RANGE_DEG*50U;
    c->fraction+=(uint32_t)c->speed*SERVO_NOMINAL_SPAN_US; step=c->fraction/denom; c->fraction%=denom;
    remaining=c->target>c->current?c->target-c->current:c->current-c->target;
    if (step>remaining) step=remaining;
    Write(i,(uint16_t)(c->target>c->current?c->current+step:c->current-step));
  }
}
uint8_t Servo_Set(uint16_t first,uint16_t second) {
  if (!ready || fault || !Destination(0U,first) || !Destination(1U,second) || !channels[0].current || !channels[1].current) return 0U;
  Target(0U,first); Target(1U,second); return 1U;
}
void Servo_ShowStatus(void) {
  App_Print("servo capabilities=preset-s2-v1\r\n");
  App_Log("servo ready=%u fault=%u S1=%u us S2=%u us busy=%u\r\n",(unsigned)ready,(unsigned)fault,
    (unsigned)channels[0].current,(unsigned)channels[1].current,(unsigned)Servo_IsBusy());
  App_Log("s1 target=%u us speed=%u deg/s bounds=%u..%u us\r\n",(unsigned)channels[0].target,
    (unsigned)channels[0].speed,(unsigned)claw_open,(unsigned)SERVO_CLAW_HARD_MAX_US);
  App_Log("s2 target=%u us speed=%u deg/s\r\n",(unsigned)channels[1].target,(unsigned)channels[1].speed);
  App_Log("tray 1=%u 2=%u 3=%u us\r\n",(unsigned)tray_positions[0],(unsigned)tray_positions[1],(unsigned)tray_positions[2]);
  App_Log("claw open=%u close=%u us\r\n",(unsigned)claw_open,(unsigned)claw_close);
}
void Servo_Command(const char *command) {
  char line[64],*args[6],*cursor; unsigned n=0U,i; uint32_t a,b,c;
  if (!command || strlen(command)>=sizeof(line)) goto invalid;
  memcpy(line,command,strlen(command)+1U); cursor=line;
  while (*cursor) {
    while (*cursor==' ' || *cursor=='\t') ++cursor;
    if (!*cursor) break;
    if (n==6U) goto invalid;
    args[n++]=cursor;
    while (*cursor && *cursor!=' ' && *cursor!='\t') {
      if (*cursor>='A' && *cursor<='Z') *cursor=(char)(*cursor-'A'+'a');
      ++cursor;
    }
    if (*cursor) *cursor++='\0';
  }
  if (!n) goto invalid;
  if (n==1U && !strcmp(args[0],"status")) { Servo_ShowStatus(); return; }
  if (n==2U && !strcmp(args[0],"preset") && !strcmp(args[1],"export")) {
    App_Log("#define SERVO_TRAY_1_US %uU\r\n#define SERVO_TRAY_2_US %uU\r\n#define SERVO_TRAY_3_US %uU\r\n",
      (unsigned)tray_positions[0],(unsigned)tray_positions[1],(unsigned)tray_positions[2]); return;
  }
  if (n==1U && !strcmp(args[0],"help")) {
    App_Print("s1 open/close | s2 1/2/3 | claw/tray aliases\r\nspeed s1/s2 DEG_S | pulse s1/s2 US | status/stop\r\nsetup s1/s2 US unloaded (DISCONNECT LINKAGE)\r\npreset s2 US1 US2 US3 | preset export (RAM only, no motion)\r\n"); return;
  }
  if (n==1U && !strcmp(args[0],"stop")) { Servo_Stop(); return; }
  if (!ready || fault) { App_Print("Servo PWM fault; inspect/reset.\r\n"); return; }
  if (n==5U && !strcmp(args[0],"preset") && !strcmp(args[1],"s2") && !Servo_IsBusy() &&
      App_ParseU32(args[2],SERVO_MIN_US,SERVO_MAX_US,&a) &&
      App_ParseU32(args[3],SERVO_MIN_US,SERVO_MAX_US,&b) &&
      App_ParseU32(args[4],SERVO_MIN_US,SERVO_MAX_US,&c) && a<b && b<c) {
    /* Atomic RAM update; do not change PWM, targets, references or S1 limits. */
    tray_positions[0]=(uint16_t)a; tray_positions[1]=(uint16_t)b; tray_positions[2]=(uint16_t)c;
    App_Print("RAM S2 presets updated; output unchanged.\r\n"); Servo_ShowStatus(); return;
  }
  if (n==2U && (!strcmp(args[0],"claw") || !strcmp(args[0],"s1"))) {
    if (!strcmp(args[1],"open")) Target(0U,claw_open);
    else if (!strcmp(args[1],"close")) Target(0U,claw_close);
    else goto invalid;
  } else if (n==2U && (!strcmp(args[0],"tray") || !strcmp(args[0],"s2")) && App_ParseU32(args[1],1U,3U,&a)) {
    Target(1U,tray_positions[a-1U]);
  } else {
    if (n==2U) i=0U;
    else if (n==3U || n==4U) i=1U;
    else goto invalid;
    if (!strcmp(args[i],"s1")) i=0U;
    else if (!strcmp(args[i],"s2")) i=1U;
    else goto invalid;
    if (n==3U && !strcmp(args[0],"pulse") && App_ParseU32(args[2],SERVO_MIN_US,SERVO_MAX_US,&a)) {
      Target(i,(uint16_t)a);
    } else if (n==3U && !strcmp(args[0],"speed") && App_ParseU32(args[2],1U,360U,&a)) {
      channels[i].speed=(uint16_t)a; channels[i].fraction=0U;
    } else if (n==4U && !strcmp(args[0],"setup") && !strcmp(args[3],"unloaded") &&
        !channels[i].current && !Servo_IsBusy() && App_ParseU32(args[2],SERVO_MIN_US,SERVO_MAX_US,&a) && Destination(i,a)) {
      /* Commissioning ONLY, linkage disconnected: immediate PWM, no homing
       * or first-move speed guarantee. Repeated setup may not bypass a ramp. */
      Write(i,(uint16_t)a); channels[i].target=(uint16_t)a; last_tick=HAL_GetTick();
    } else goto invalid;
  }
  Servo_ShowStatus(); return;
invalid:
  App_Print("Invalid servo args.\r\n");
}

uint8_t Servo_ClawReady(void) { return ready && !fault && channels[0].current && !Servo_IsBusy(); }
uint8_t Servo_Claw(uint8_t close) {
  if (!Servo_ClawReady()) return 0U;
  Target(0U,close?claw_close:claw_open); return 1U;
}
uint8_t Servo_Tray(uint8_t slot) {
  if (!Servo_ClawReady() || !channels[1].current || slot<1U || slot>3U) return 0U;
  Target(1U,tray_positions[slot-1U]);
  return 1U;
}
