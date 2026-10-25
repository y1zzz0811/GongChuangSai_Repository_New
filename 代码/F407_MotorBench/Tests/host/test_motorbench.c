#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../Core/Src/motor_bench.c"

typedef struct { int enabled,pending,moving; uint8_t direction; uint32_t pulses,rpm,end; int64_t pos; } Model;
static Model models[8];
static UART_HandleTypeDef pc={1,NULL},driver={2,NULL};
static uint32_t clock_ms;
static int transmissions,triggers,broadcast_stops,drop_address,reject_load,bad_config,noise,stale,stall,wrong_distance;
static uint8_t seen_dir[16][4];
static uint32_t seen_pulses[16][4],seen_rpm[16][4];
static char console_log[100000];
static size_t log_size;

uint32_t HAL_GetTick(void) { return clock_ms; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
  assert(n==1); u->rx=p; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { u->rx=NULL; return HAL_OK; }
static void feed(UART_HandleTypeDef *u,const uint8_t *data,size_t size) {
  while (size--) { assert(u->rx); *u->rx=*data++; HAL_UART_RxCpltCallback(u); }
}
static void reply(const uint8_t *data,size_t size) {
  if (noise) { uint8_t other[]={7,0xFD,0x9F,0x6B,1,0xFD,0x9F,0x6B,0x55}; feed(&driver,other,sizeof(other)); }
  feed(&driver,data,size);
}
static void update_models(void) {
  int i;
  for (i=1;i<=7;++i) if (models[i].moving && (int32_t)(clock_ms-models[i].end)>=0 && !stale) {
    int64_t delta=((int64_t)models[i].pulses*65536+CHASSIS_PULSES_PER_REV/2)/CHASSIS_PULSES_PER_REV;
    if (wrong_distance) delta/=2;
    models[i].pos += models[i].direction ? -delta : delta;
    models[i].moving=0;
  }
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *f,uint16_t n,uint32_t timeout) {
  uint8_t out[64]={0}; uint32_t mag; int i,a; (void)timeout;
  if (u==&pc) { assert(log_size+n<sizeof(console_log)); memcpy(console_log+log_size,f,n); log_size+=n; console_log[log_size]=0; return HAL_OK; }
  ++transmissions; assert(n>=3); a=f[0];
  if (!a) {
    if (f[1]==0xFE) { ++broadcast_stops; for (i=1;i<=7;++i) models[i].moving=models[i].pending=0; }
    if (f[1]==0xFF) {
      assert(triggers<16);
      for (i=1;i<=4;++i) {
        assert(models[i].pending); seen_dir[triggers][i-1]=models[i].direction;
        seen_pulses[triggers][i-1]=models[i].pulses;
        seen_rpm[triggers][i-1]=models[i].rpm;
        models[i].pending=0; models[i].moving=1; models[i].end=clock_ms+20;
      } ++triggers;
    }
    return HAL_OK;
  }
  if (a<1 || a>7 || a==drop_address) return HAL_OK;
  out[0]=(uint8_t)a; out[1]=f[1];
  switch (f[1]) {
    case 0xFE: models[a].moving=models[a].pending=0; out[2]=2; out[3]=0x6B; reply(out,4); break;
    case 0x42:
      out[2]=0x21; out[3]=0x15; out[4]=bad_config?0x32:0x19; out[6]=2;
      out[13]=0x6B; out[9]=(uint8_t)CHASSIS_MICROSTEPS; out[21]=0; out[22]=1; out[32]=0x6B; reply(out,33); break;
    case 0x1A: out[2]=0; out[3]=6; out[4]=0x6B; reply(out,5); break;
    case 0xF3: models[a].enabled=1; out[2]=2; out[3]=0x6B; reply(out,4); break;
    case 0xFD:
      assert(n==13 && f[10]==2 && f[11]==1 && f[12]==0x6B);
      if (a==reject_load) { out[2]=0xE2; out[3]=0x6B; reply(out,4); break; }
      models[a].pending=1; models[a].direction=f[2];
      models[a].rpm=((uint32_t)f[3]<<8)|f[4];
      models[a].pulses=((uint32_t)f[6]<<24)|((uint32_t)f[7]<<16)|((uint32_t)f[8]<<8)|f[9];
      out[2]=2; out[3]=0x6B; reply(out,4); break;
    case 0x3A:
      out[2]=(uint8_t)((models[a].enabled?1:0)|((!models[a].moving || stale)?2:0));
      if (stall && a==3 && triggers) out[2]|=0x0C;
      out[3]=0x6B; reply(out,4); break;
    case 0x36:
      out[2]=models[a].pos<0; mag=(uint32_t)(models[a].pos<0?-models[a].pos:models[a].pos);
      out[3]=(uint8_t)(mag>>24); out[4]=(uint8_t)(mag>>16); out[5]=(uint8_t)(mag>>8); out[6]=(uint8_t)mag;
      out[7]=0x6B; reply(out,8); break;
    case 0x1F: out[2]=0; out[3]=206; out[4]=0x22; out[5]=0x14; out[6]=0x6B; reply(out,7); break;
    default: break;
  }
  return HAL_OK;
}
static void tick(unsigned milliseconds) {
  while (milliseconds--) { ++clock_ms; update_models(); MotorBench_Poll(); }
}
static void command(const char *s) { feed(&pc,(const uint8_t *)s,strlen(s)); tick(4); }
static void reset(void) {
  memset(models,0,sizeof(models)); memset(seen_dir,0,sizeof(seen_dir)); memset(seen_pulses,0,sizeof(seen_pulses)); memset(seen_rpm,0,sizeof(seen_rpm));
  clock_ms=0; transmissions=triggers=broadcast_stops=drop_address=reject_load=bad_config=noise=stale=stall=wrong_distance=0;
  log_size=0; MotorBench_Init(&pc,&driver);
  assert(transmissions==0 && route.phase==IDLE && length_mm==190 && width_mm==215);
  assert(DistancePulses(1000)==12732 && TurnPulses(90)==4050);
  assert(linear_rpm==CHASSIS_DEFAULT_LINEAR_RPM && turn_rpm==CHASSIS_DEFAULT_TURN_RPM);
}
static void until_terminal(unsigned limit) {
  while (limit-- && Busy()) tick(1);
  assert(!Busy());
}
int main(void) {
  setvbuf(stdout,NULL,_IONBF,0);
  int i,j,before;
  reset(); command("square 1000\n"); until_terminal(20000); assert(route.phase==IDLE && triggers==8);
  for (i=0;i<8;++i) for (j=0;j<4;++j) {
    static const uint8_t f[]={1,0,0,1},r[]={0,0,0,0};
    assert(seen_dir[i][j]==((i&1)?r[j]:f[j])); assert(seen_pulses[i][j]==((i&1)?4050U:12732U));
  }
  puts("PASS full square: 8 synchronized moves, correct directions and pulse counts");
  reset(); command("speed linear 30\n"); command("speed turn 10\n");
  command("square 1000\n"); until_terminal(20000); assert(route.phase==IDLE && triggers==8);
  for (i=0;i<8;++i) for (j=0;j<4;++j) assert(seen_rpm[i][j]==((i&1)?10U:30U));
  puts("PASS square uses independent translation/turn RPM in all four transmitted motor frames");
  reset(); command("speed 25\n"); assert(linear_rpm==25 && turn_rpm==25);
  command("speed linear 20\n"); assert(linear_rpm==20 && turn_rpm==25);
  command("speed turn 5\n"); assert(linear_rpm==20 && turn_rpm==5);
  command("status\n"); assert(strstr(console_log,"linear-rpm=20 turn-rpm=5"));
  command("speed linear 0\n"); command("speed turn 101\n"); command("speed linear\n");
  command("speed other 5\n"); command("speed 0\n"); command("speed 101\n");
  command("speed 10 extra\n"); command("speed linear 10 extra\n");
  assert(linear_rpm==20 && turn_rpm==5 && transmissions==0);
  command("backward 100\n"); while (route.phase!=BASELINE) tick(1);
  assert(route.rpm==20 && route.deadline-clock_ms==((uint64_t)route.pulses*60000U/(20U*CHASSIS_PULSES_PER_REV))*3U+10000U);
  command("speed linear 40\n"); command("speed turn 40\n"); command("speed 40\n");
  assert(linear_rpm==20 && turn_rpm==5 && route.rpm==20);
  until_terminal(5000); assert(route.phase==IDLE && triggers==1);
  for (j=0;j<4;++j) assert(seen_rpm[0][j]==20);
  command("turn left 90\n"); while (route.phase!=BASELINE) tick(1);
  assert(route.rpm==5 && route.deadline-clock_ms==((uint64_t)route.pulses*60000U/(5U*CHASSIS_PULSES_PER_REV))*3U+10000U);
  until_terminal(5000); assert(route.phase==IDLE && triggers==2);
  for (j=0;j<4;++j) assert(seen_rpm[1][j]==5);
  command("left 100\n"); until_terminal(5000);
  command("right 100\n"); until_terminal(5000);
  for (i=2;i<4;++i) for (j=0;j<4;++j) assert(seen_rpm[i][j]==20);
  command("speed 1\n"); assert(linear_rpm==1 && turn_rpm==1);
  command("speed 100\n"); assert(linear_rpm==100 && turn_rpm==100);
  reset();
  puts("PASS speed isolation, legacy command, limits, busy rejection, step deadlines and reboot defaults");
  {
    static const char *moves[]={"forward 100\n","backward 100\n","left 100\n","right 100\n","turn left 30\n","turn right 30\n"};
    static const uint8_t expected[6][4]={{1,0,0,1},{0,1,1,0},{1,1,0,0},{0,0,1,1},{1,1,1,1},{0,0,0,0}};
    /* Independent measured physical order FL=M2, FR=M1, RL=M3, RR=M4. */
    static const unsigned physical_address_index[4]={1,0,2,3};
    static const uint8_t measured_forward[4]={1,0,0,1};
    static const int chassis_expected[6][3]={{4,0,0},{-4,0,0},{0,4,0},{0,-4,0},{0,0,4},{0,0,-4}};
    for (i=0;i<6;++i) {
      int wheel[4],vx,vy,omega;
      reset(); command(moves[i]); until_terminal(5000);
      assert(route.phase==IDLE && triggers==1 && !memcmp(seen_dir[0],expected[i],4));
      for (j=0;j<4;++j) {
        unsigned address_index=physical_address_index[j];
        wheel[j]=seen_dir[0][address_index]==measured_forward[address_index]?1:-1;
      }
      vx=wheel[0]+wheel[1]+wheel[2]+wheel[3];
      vy=-wheel[0]+wheel[1]+wheel[2]-wheel[3];
      omega=-wheel[0]+wheel[1]-wheel[2]+wheel[3];
      assert(vx==chassis_expected[i][0] && vy==chassis_expected[i][1] && omega==chassis_expected[i][2]);
      command("status\n"); assert(strstr(console_log,"wheel addresses FL=2 FR=1 RL=3 RR=4"));
    }
  }
  puts("PASS actual wheel mapping: six motion frames and independent chassis translation/yaw reconstruction");
  reset(); noise=1; command("turn left 90\n"); until_terminal(5000); assert(route.phase==IDLE && triggers==1);
  { uint8_t wanted[]={1,1,1,1}; assert(!memcmp(seen_dir[0],wanted,4)); }
  puts("PASS unrelated asynchronous packets do not replace acknowledgements");
  reset(); command("geometry 200 250\n"); assert(TurnPulses(90)==4500); command("cal turn 1100\n"); assert(TurnPulses(90)==4950);
  command("cal straight 1100\n"); assert(DistancePulses(1000)==14006); reset(); assert(TurnPulses(90)==4050);
  command("speed 0\n"); command("forward -100\n"); command("square 3001\n"); assert(!transmissions);
  puts("PASS configuration, calibration, reboot defaults and argument validation");
  reset(); drop_address=2; command("start\n"); until_terminal(5000); assert(route.phase==FAULT && triggers==0 && broadcast_stops);
  before=transmissions; command("start\n"); assert(transmissions==before);
  command("stopall\n"); until_terminal(5000); assert(route.phase==FAULT);
  drop_address=0; command("clearfault\n");
  command("forward 100\n"); until_terminal(5000); assert(route.phase==IDLE && triggers==1);
  puts("PASS missing motor latches fault, blocks route, allows explicit recovery");
  reset(); bad_config=1; command("start\n"); until_terminal(5000); assert(route.phase==FAULT && triggers==0);
  reset(); reject_load=3; command("start\n"); until_terminal(5000); assert(route.phase==FAULT && triggers==0);
  puts("PASS invalid configuration and rejected preload never trigger motion");
  reset(); stale=1; command("square 100\n"); until_terminal(30000); assert(route.phase==FAULT && triggers==1);
  reset(); wrong_distance=1; command("square 100\n"); until_terminal(30000); assert(route.phase==FAULT && triggers==1);
  puts("PASS stale arrived flags and wrong displacement cannot advance route");
  reset(); stall=1; command("start\n"); until_terminal(5000); assert(route.phase==FAULT && triggers==1 && broadcast_stops);
  puts("PASS stall aborts and broadcasts stop");
  reset(); command("start\n"); while (!triggers) tick(1); before=transmissions;
  command("tx 01 1F 6B\n"); assert(transmissions==before); command("status\n"); assert(Busy());
  command("! "); assert(broadcast_stops); until_terminal(5000); assert(route.phase==IDLE && triggers==1);
  puts("PASS busy rejects raw commands, status works, ! stops without newline");
  reset(); command("start\n"); while (!triggers) tick(1); command("stop\n"); until_terminal(5000); assert(route.phase==IDLE && broadcast_stops);
  reset(); command("start\n"); while (!triggers) tick(1); HAL_UART_ErrorCallback(&driver); tick(1); until_terminal(5000); assert(route.phase==FAULT);
  puts("PASS legacy stop cancels running route; UART errors abort");
  reset(); command("id 6\n"); command("readpos\n"); tick(30); assert(strstr(console_log,"RX (8): 06 36"));
  command("tx 06 1F 6B\n"); tick(30); assert(strstr(console_log,"06 1F 00 CE"));
  puts("PASS legacy id/readpos/tx remain available for other motor addresses");
  reset(); clock_ms=0xFFFFFF00U; command("forward 100\n"); until_terminal(5000); assert(route.phase==IDLE && triggers==1);
  reset(); { uint8_t data[600]; memset(data,'x',sizeof(data)); feed(&pc,data,sizeof(data)); }
  tick(1); until_terminal(5000); assert(route.phase==FAULT && broadcast_stops);
  puts("PASS clock rollover and receive overflow fail-safe");
  reset(); command("\xE5\x90\xAF\xE5\x8A\xA8\n"); until_terminal(20000); assert(triggers==8 && route.phase==IDLE);
  puts("PASS UTF-8 Chinese start alias; ALL HOST TESTS PASSED");
  return 0;
}
