#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "competition.h"
#include "motor_bench.h"
#include "arm_bench.h"
#include "app_console.h"
#include "mission.h"
#include "servo.h"
#include "route_library.h"
#include "route_scene.h"
#include "route2.h"
#include "servo_config.h"
#include "arm_units.h"
#define CLAW_CLOSE SERVO_CLAW_CLOSE_US
#include "qr_bridge.h"
typedef struct { uint8_t enabled,pending,moving,dir; uint32_t pulses,rpm,end; int64_t pos; } Model;
static Model models[8];
static UART_HandleTypeDef pc={1,NULL},chassis={2,NULL},arm={6,NULL};
static UART_HandleTypeDef pi={5,NULL},hmi={3,NULL};
static TIM_HandleTypeDef servo_timer;
static unsigned arm_n,arm_ids[192],arm_dirs[192],arm_pulses[192],arm_claw[192],arm_station[192];
static uint32_t now;
static unsigned sent[7],triggers[7],stops[7],moves[8],disables;
static uint8_t drop_id,reject_id,stall_id,noise,stale;
static unsigned realistic_duration,monitor_running,running_writes;
static unsigned shorten_lift,drift_before_turn;
static unsigned hmi_blank;
static uint8_t wide_trial;
static char pi_request[80];
static char output[1000000]; static size_t used;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) { assert(n==1); u->rx=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { u->rx=NULL; return HAL_OK; }
static void feed(UART_HandleTypeDef *u,const uint8_t *p,size_t n) {
  while (n--) { assert(u->rx); *u->rx=*p++; HAL_UART_RxCpltCallback(u); }
}
static void reply(UART_HandleTypeDef *u,uint8_t *p,size_t n) {
  if (noise) { const uint8_t junk[]={99,0xFD,2,0x6B,88,0,0x55,0x6B}; feed(u,junk,sizeof(junk)); }
  feed(u,p,n);
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *f,uint16_t n,uint32_t timeout) {
  uint8_t r[64]={0}; unsigned i,lo=u==&chassis?1:5,hi=u==&chassis?4:7; uint32_t mag;
  (void)timeout;
  if (u==&pc) { assert(used+n<sizeof(output)); memcpy(output+used,f,n); used+=n; output[used]=0; return HAL_OK; }
  if (u==&hmi) { if (n>=11U && !memcmp(f,"t0.txt=\"---",11U)) ++hmi_blank; return HAL_OK; }
  if (u==&pi) { assert(n<sizeof(pi_request)); memcpy(pi_request,f,n); pi_request[n]=0; return HAL_OK; }
  assert(u==&chassis || u==&arm); assert(n>=3); ++sent[u->id];
  if (monitor_running && u==&chassis && models[1].moving && f[1]!=0x3AU && f[1]!=0x36U) ++running_writes;
  if (!f[0]) {
    if (f[1]==0xFE) { ++stops[u->id]; for (i=lo;i<=hi;++i) models[i].moving=0; }
    else if (f[1]==0xFF) {
      ++triggers[u->id];
      for (i=lo;i<=hi;++i) {
        assert(models[i].pending); models[i].pending=0; models[i].moving=1;
        models[i].end=now+((realistic_duration && u==&chassis)?(uint32_t)(((uint64_t)models[i].pulses*60000U+models[i].rpm*3200U-1U)/(models[i].rpm*3200U)):20U);
      }
    } else assert(0);
    return HAL_OK;
  }
  assert(f[0]>=lo && f[0]<=hi); i=f[0]; if (i==drop_id) return HAL_OK;
  r[0]=(uint8_t)i; r[1]=f[1];
  switch (f[1]) {
    case 0x42: r[2]=0x21; r[3]=0x15; r[4]=0x19; r[6]=2; r[9]=16; r[10]=1; r[22]=1; r[32]=0x6B; reply(u,r,33); break;
    case 0x1A: r[3]=6; r[4]=0x6B; reply(u,r,5); break;
    case 0xFE: models[i].moving=0; r[2]=2; r[3]=0x6B; reply(u,r,4); break;
    case 0xF3: assert(n==6); if (!f[3]) ++disables; models[i].enabled=f[3]; r[2]=2; r[3]=0x6B; reply(u,r,4); break;
    case 0xFD:
      if (u==&arm) {
        assert(arm_n<sizeof(arm_ids)/sizeof(arm_ids[0])); arm_ids[arm_n]=i; arm_dirs[arm_n]=f[2];
        arm_claw[arm_n]=servo_timer.compare[0]; arm_station[arm_n]=triggers[2];
        arm_pulses[arm_n++]=((uint32_t)f[6]<<24)|((uint32_t)f[7]<<16)|((uint32_t)f[8]<<8)|f[9];
      }
      assert(n==13 && f[10]==2 && f[12]==0x6B);
      r[2]=(uint8_t)(i==reject_id?0xE2:2); r[3]=0x6B;
      if (i!=reject_id) {
        models[i].pulses=((uint32_t)f[6]<<24)|((uint32_t)f[7]<<16)|((uint32_t)f[8]<<8)|f[9];
        models[i].rpm=((uint32_t)f[3]<<8)|f[4]; models[i].dir=f[2]; ++moves[i];
        if (f[11]) models[i].pending=1;
        else { models[i].moving=1; models[i].end=now+20U; }
      }
      reply(u,r,4); break;
    case 0x36:
      if (drift_before_turn && i==6U && strstr(output,"WORK group=2 step=11 axis=0")) {
        models[i].pos+=8192; drift_before_turn=0U;
      }
      mag=(uint32_t)(models[i].pos<0?-models[i].pos:models[i].pos); r[2]=(uint8_t)(models[i].pos<0);
      r[3]=(uint8_t)(mag>>24); r[4]=(uint8_t)(mag>>16); r[5]=(uint8_t)(mag>>8); r[6]=(uint8_t)mag; r[7]=0x6B; reply(u,r,8); break;
    case 0x3A:
      r[2]=(uint8_t)((models[i].enabled?1U:0U)|(!models[i].moving?2U:0U)|(i==stall_id?12U:0U)); r[3]=0x6B; reply(u,r,4); break;
    case 0x1F: r[2]=0; r[3]=206; r[4]=0x22; r[5]=0x14; r[6]=0x6B; reply(u,r,7); break;
    default: assert(0);
  }
  return HAL_OK;
}
static void tick(unsigned n) {
  while (n--) {
    unsigned i; ++now;
    for (i=1;i<8;++i) if (models[i].moving && (int32_t)(now-models[i].end)>=0 && !stale) {
      int64_t delta=((int64_t)models[i].pulses*65536+1600)/3200;
      if (shorten_lift && i==6U && arm_n==11U) delta/=8;
      models[i].pos+=models[i].dir?-delta:delta; models[i].moving=0;
    }
    Competition_Poll();
  }
}
static void cmd(const char *s) { feed(&pc,(const uint8_t *)s,strlen(s)); feed(&pc,(const uint8_t *)"\n",1U); tick(8); }
/* Mock-only output commissioning; never touches a physical serial device. */
static void setup_servos(unsigned first,unsigned second) {
  char text[48]; snprintf(text,sizeof(text),"servo setup s1 %u unloaded",first); cmd(text);
  snprintf(text,sizeof(text),"servo setup s2 %u unloaded",second); cmd(text);
}
static void settle(void) {
  unsigned n=10000;
  while (n-- && (MotorBench_IsBusy() || ArmBench_IsBusy())) tick(1);
  assert(!MotorBench_IsBusy() && !ArmBench_IsBusy()); tick(8);
}
static void settle_servo(void) {
  unsigned n=20000U;
  while (n-- && Servo_IsBusy()) tick(1);
  assert(!Servo_IsBusy());
}
static void reset(void) {
  wide_trial=0U;
  arm_n=0U;
  memset(models,0,sizeof(models)); memset(sent,0,sizeof(sent)); memset(triggers,0,sizeof(triggers));
  memset(stops,0,sizeof(stops)); memset(moves,0,sizeof(moves)); used=now=disables=0;
  drop_id=reject_id=stall_id=noise=stale=0;
  realistic_duration=monitor_running=running_writes=0U;
  shorten_lift=drift_before_turn=0U;
  hmi_blank=0U;
  pi_request[0]=0;
  Competition_Init(&pc,&chassis,&arm);
  QrBridge_Init(&pi,&hmi);
  memset(&servo_timer,0,sizeof(servo_timer)); servo_timer.Init.Prescaler=83U; servo_timer.Init.Period=19999U; Servo_Init(&servo_timer);
  assert(!sent[2] && !sent[6] && pc.rx && chassis.rx && arm.rx);
}
static uint32_t ready_mask;
static unsigned begins,cancels,mode;
static MissionRequest history[64];
static uint32_t ready(void) { return ready_mask; }
static PortResult begin(const MissionRequest *r) {
  assert(begins<64); history[begins++]=*r;
  if (r->op==READ_QR && mode!=2U) assert(Mission_AcceptCode("156+123+516+231"));
  return mode==1U?PORT_WAIT:PORT_DONE;
}
static PortResult poll_port(const MissionRequest *r) { (void)r; return PORT_WAIT; }
static void cancel(void) { ++cancels; }
static void connect_ports(unsigned m) {
  MissionPorts p={ready,begin,poll_port,cancel}; ready_mask=SERVICE_ALL; mode=m; begins=cancels=0; assert(Mission_RegisterPorts(&p));
}
static void wait_text(const char *text) {
  unsigned n=20000U;
  while (n-- && !strstr(output,text)) tick(1);
  if (!strstr(output,text)) fprintf(stderr,"Waiting for %s; last output: %s\n",text,output+(used>2000U?used-2000U:0U));
  assert(strstr(output,text));
}
static void scan_route(void) {
  const char frame[]="QR,253+231+532+132\n";
  feed(&pi,(const uint8_t *)frame,sizeof(frame)-1U); tick(8);
}
static void prepare_reference(void) {
  cmd("arm ref set"); settle();
  assert(strstr(output,"ARM REF M7 zero=+0 counts"));
  models[7].pos=-67809; /* fixture: 3311 pulses forward after capture */
}
static void raw_station(void) {
  prepare_reference();
  cmd("route1 start"); wait_text("ROUTE1: check QR"); scan_route(); wait_text("ROUTE1 raw");
}
static void prepare_route2(void) {
  setup_servos(500U,1500U);
  cmd("arm on");
  prepare_reference();
  cmd("obs none");
}
static void route_scan(uint8_t id) {
  const char frame[]="QR,156+123+516+231\n";
  const char stale_frame[]="QR,253+231+532+132\n";
  feed(&pi,(const uint8_t *)stale_frame,sizeof(stale_frame)-1U);
  tick(8);
  cmd(id==3U?"route start 3":"route start 2");
  wait_text("STEP 2/2: forward 1050");
  feed(&pi,(const uint8_t *)frame,sizeof(frame)-1U);
  tick(8);
}
static void vision_reply(unsigned id,unsigned ring,int forward,int left,unsigned age,unsigned corrupt,uint8_t top) {
  char line[80];
  unsigned crc=0U;
  int n=snprintf(line,sizeof(line),"VR,%u,%u,10,%u,%d,%d%s",id,ring,age,forward,left,
                 top==2U?",RAW":top?",TOP":"");
  for (int i=0;i<n;++i) {
    crc^=(unsigned char)line[i];
    for (unsigned j=0;j<8U;++j) crc=((crc<<1)^((crc&128U)?7U:0U))&255U;
  }
  snprintf(line+n,sizeof(line)-(size_t)n,",%02X\n",crc^corrupt);
  feed(&pi,(const uint8_t *)line,strlen(line));
}

static void route2_vision_trial(unsigned scenario) {
  unsigned cursor=0,requests=0,done=0;
  reset(); prepare_route2();
  cmd("route2 vision on");
  if (scenario!=1) cmd("route2 limits 20 10 10");
  route_scan(2U);
  for (unsigned t=0;t<60000U && Mission_IsRunning() && !done;++t) {
    const char *event;
    unsigned id,ring;
    tick(1);
    event=strstr(output+cursor,"@route2 ");
    if (event) {
      char command[48];
      assert(sscanf(event,"@route2 %u",&id)==1);
      if (strstr(event," done place_rough ") &&
          strstr(event," done place_rough ") < strchr(event,'\n')) done=1;
      cursor=(unsigned)(strchr(event,'\n')-output+1);
      snprintf(command,sizeof(command),"route2 ack %u ok",id); cmd(command);
    }
    if (pi_request[0]) {
      assert(sscanf(pi_request,"VQ,%u,%u",&id,&ring)==2 && ring==1);
      pi_request[0]=0; ++requests;
      if (scenario==2) {
        vision_reply(id+1U,ring,0,0,0,0,0);
        vision_reply(id,2,0,0,0,0,0);
        vision_reply(id,ring,0,0,1501,0,0);
        vision_reply(id,ring,0,0,0,1,0);
      } else {
        int left=requests==1?6000:5000,forward=requests<=2?-5000:2000;
        if (scenario==3) { left=0; forward=requests==2?3000:0; }
        vision_reply(id,ring,forward,left,0,0,0);
      }
    }
  }
  if (scenario==0) {
    assert(done && requests==4U && moves[7]==4U);
    assert(strstr(output,"left 6 mm") && strstr(output,"rpm=10"));
    assert(models[7].pos==-67809);
  } else {
    assert(!done && Mission_GetState()==MISSION_FAULT);
    if (scenario==2) assert(strstr(output,"vision timeout") && requests==1U);
    else assert(strstr(output,"vision correction blocked"));
    assert(!moves[7] && !strstr(output,"left 6 mm"));
  }
}
static void run_route2(uint8_t visual,uint8_t no_lift) {
  unsigned wide_views=0U;
  int64_t starting_rail=models[7].pos;
  static const char *const ops[]={"load_raw","place_rough","reload_rough","place_temp"};
  static const unsigned colors[2][3]={{1,5,6},{5,1,6}};
  static const unsigned rough[2][3]={{1,2,3},{2,3,1}};
  static const unsigned temp[2][3]={{1,2,3},{2,1,3}};
  static const unsigned servo_slots[]={850,1500,2080};
  unsigned cursor=0U,items=0U,ready=0U,bases=0U,home=0U;
  unsigned top_count=0U,ring_count=0U,raw_count=0U,reload_trim=0U,stack_views[3]={0};
  for(unsigned tick_count=0;tick_count<400000U && Mission_IsRunning();++tick_count) {
    const char *event;
    tick(1);
    if (pi_request[0]) {
      unsigned id,ring,batch=items/12U,stage=items%12U/3U,index=items%3U;
      uint8_t top=(uint8_t)(strstr(pi_request,",TOP")!=NULL);
      assert(visual && ready==items+1U);
      assert(sscanf(pi_request,"VQ,%u,%u",&id,&ring)==2);
      if (stage==0U) {
        unsigned before=arm_n;
        assert(strstr(pi_request,",RAW") && ring==colors[batch][index]);
        pi_request[0]=0;
        vision_reply(id,ring,0,0,0,0,1); tick(20);
        assert(arm_n==before);
        vision_reply(id,ring,2000,5000,0,0,2);
        ++raw_count;
        continue;
      }
      assert(ring==(stage==3U?temp[batch][index]:rough[batch][index]));
      assert(top==(stage==2U || (stage==3U && batch==1U)));
      pi_request[0]=0;
      if (top) {
        ++top_count;
        if (stage==2U && !reload_trim) {
          /* A floor reply with the correct ID and ring must not start a pickup. */
          unsigned before=arm_n;
          vision_reply(id,ring,0,0,0,0,0); tick(20);
          assert(arm_n==before);
          ++reload_trim;
          vision_reply(id,ring,-5000,0,0,0,1);
          continue;
        }
        if (stage==3U) ++stack_views[index];
      } else ++ring_count;
      if(wide_trial && batch==0U && stage==1U && index==0U && wide_views<3U) {
        vision_reply(id,ring,wide_views==0U?45000:15000,wide_views<2U?-60000:0,0,0,top);
        ++wide_views;
        continue;
      }
      vision_reply(id,ring,0,0,0,0,top);
    }
    event=strstr(output+cursor,"@route2 ");
    if(event) {
      unsigned id,b,i,c,r,layer,slot;
      char type[16],op[32],command[48],locked[16];
      assert(sscanf(event,"@route2 %u %15s %31s %u %u %u %u %u %u",&id,type,op,&b,&i,&c,&r,&layer,&slot)==9);
      cursor=(unsigned)(strchr(event,'\n')-output+1);
      if(!strcmp(type,"ready") || !strcmp(type,"done")) {
        unsigned batch=items/12U,stage=items%12U/3U,index=items%3U;
        assert(b==batch+1U && i==index+1U && slot==i && c==colors[batch][index]);
        assert(!strcmp(op,ops[stage]) && servo_timer.compare[1]==servo_slots[index]);
        assert(r==(stage==0U?0U:stage==3U?temp[batch][index]:rough[batch][index]));
        assert(layer==(batch==1U && stage==3U?2U:1U));
        if(!strcmp(type,"ready")) {
          assert(ready==items);
          if(!items) {
            unsigned before=arm_n;
            const char newer[]="QR,253+231+532+132\n";
            feed(&pi,(const uint8_t *)newer,sizeof(newer)-1U);
            cmd("route next");
            cmd("route2 ack 99 ok");
            tick(50);
            assert(arm_n==before && QrBridge_GetCode(locked,NULL,NULL) && !strcmp(locked,"156+123+516+231"));
          }
          ++ready;
        } else {
          assert(ready==items+1U);
          ++items;
        }
      } else if(!strcmp(type,"base")) {
        assert(items==3U || items==9U || items==12U || items==15U || items==21U || items==24U);
        ++bases;
      } else {
        assert(!strcmp(type,"home") && !strcmp(op,"go_home") && items==24U);
        ++home;
      }
      snprintf(command,sizeof(command),"route2 ack %u ok",id);
      cmd(command);
    }
  }
  MissionSnapshot s;
  Mission_GetSnapshot(&s);
  assert(s.state==MISSION_DONE && items==24U && ready==24U && bases==6U && home==1U);
  assert(s.grabs==12U && s.places==12U && !strcmp(s.task_code,"156+123+516+231"));
  assert(triggers[2]==(wide_trial?61U:58U)); /* Wide case: fore/aft + side + fore/aft restore. */
  assert(models[5].pos==0 && models[6].pos==0 && models[7].pos==starting_rail && servo_timer.compare[0]==500U);
  unsigned stack_moves=0;
  uint32_t stack_pulses;
  assert(ArmUnits_ToPulses(1U,RouteLibrary_StackDepth(),&stack_pulses));
  for(unsigned i=0;i<arm_n;++i) if (arm_ids[i]==6U && arm_pulses[i]==stack_pulses) {
    assert(arm_dirs[i]==(stack_moves%2U)); /* alternating down/up at current stack depth */
    ++stack_moves;
  }
  assert(stack_moves==(no_lift?0U:6U));
  if(no_lift) { assert(!moves[6]); for(unsigned j=0;j<arm_n;++j) assert(arm_ids[j]!=6U); }
  if (visual) {
    assert(ring_count==(wide_trial?21U:18U) && top_count==13U && raw_count==6U && moves[7]==(wide_trial?6U:2U));
    for (unsigned i=0;i<3U;++i) assert(stack_views[i]==2U);
  } else assert(arm_n==(no_lift?48U:120U) && !moves[7] && !top_count && !ring_count);
}
int main(int argc,char **argv) {
  unsigned before,i; MissionCode parsed,unchanged; char small[5],expected[80],actual[80];
  setvbuf(stdout,NULL,_IONBF,0);
  if(argc==2 && !strcmp(argv[1],"--logging-limits")) {
    reset(); before=(unsigned)used; tick(1000U); assert(used>before);
    assert(strstr(output,"@state") && strstr(output,"@r2state"));
    cmd("telemetry off"); before=(unsigned)used; tick(5000U); assert(used==before);
    cmd("status"); assert(used>before);
    reset(); prepare_route2(); wide_trial=1U;
    cmd("route2 vision on"); cmd("route2 rawvision on");
    cmd("route2 limits 100 100 20 30"); cmd("route2 rawlimit 100");
    route_scan(3U); run_route2(1U,1U);
    assert(strstr(output,"foreaft 30") && strstr(output,"shift 60") && !moves[6]);
    reset(); prepare_route2(); cmd("route2 vision on"); cmd("route2 rawvision on");
    route_scan(3U); wait_text("@route2 1 ready load_raw"); cmd("route2 ack 1 ok");
    for(unsigned t=0U;t<15000U && !pi_request[0];++t) tick(1U);
    unsigned request,target;
    assert(sscanf(pi_request,"VQ,%u,%u",&request,&target)==2);
    before=triggers[2]; pi_request[0]=0;
    vision_reply(request,target,-45000,0,0,0,2U); tick(500U); settle();
    assert(triggers[2]==before && ArmBench_HasFault() && strstr(output,"work-zone boundary"));
    puts("PASS continuous default telemetry, explicit off/on-demand status, 60 mm lateral/30 mm fore-aft formal corrections with restoration, and work-zone boundary rejection");
    return 0;
  }
  if(argc==2 && !strcmp(argv[1],"--route3")) {
    reset(); prepare_route2(); stall_id=6U; route_scan(3U); run_route2(0U,1U);
    assert(!stops[2] && !stops[6] && strstr(output,"ROUTE3 skip M6"));
    /* Completion must not leak no-lift behaviour into the next route2. */
    stall_id=0U; cmd("mission reset");
    arm_n=used=0U; output[0]=0; memset(triggers,0,sizeof(triggers)); memset(moves,0,sizeof(moves));
    route_scan(2U); run_route2(0U,0U);
    reset(); prepare_route2(); stall_id=6U; cmd("route2 vision on"); cmd("route2 rawvision on");
    cmd("route2 limits 0 0 10"); route_scan(3U); run_route2(1U,1U);
    assert(!stops[2] && !stops[6]);
    reset(); prepare_route2(); route_scan(3U); cmd("!"); settle();
    assert(stops[2] && stops[6] && !moves[6] && !Route2_IsActive() && !Route2_IsPending());
    cmd("clearfault"); assert(!ArmBench_HasFault() && !MotorBench_HasFault());
    /* M6 remains a required communication/clearance/stop participant. */
    reset(); prepare_route2(); route_scan(3U); drop_id=6U;
    wait_text("STOP: mission failed/timeout"); settle();
    assert(!moves[6] && ArmBench_HasFault());
    reset(); prepare_route2(); route_scan(3U); wait_text("@route2 1 ready load_raw");
    models[6].pos=8192; cmd("route2 ack 1 ok");
    for(unsigned t=0U;t<15000U && !ArmBench_HasFault();++t) tick(1U);
    settle();
    assert(!moves[6] && ArmBench_HasFault() && MotorBench_HasFault());
    puts("PASS route3: 24 simulated transfers with/without vision, zero M6 moves, normal route2 restored, stop ACK and M6 read/height faults retained");
    return 0;
  }
  if(argc==2 && !strcmp(argv[1],"--route2-navigation")) {
    reset(); prepare_route2(); route_scan(2U); run_route2(0U,0U);
    reset(); prepare_route2(); cmd("route2 vision on"); cmd("route2 rawvision on");
    cmd("route2 limits 0 0 10"); route_scan(2U); run_route2(1U,0U);
    assert(!stops[2] && !stops[6]);
    puts("PASS route2 navigation: two batches, 24 transfers, return home, with and without vision");
    return 0;
  }
  reset(); cmd("start"); cmd("chassis start"); cmd("\xE5\x90\xAF\xE5\x8A\xA8");
  assert(!sent[2] && !sent[6] && strstr(output,"START BLOCKED") && Mission_GetState()==MISSION_IDLE);
  puts("PASS 01 startup has three live RX channels, no motor traffic, start aliases never run square");
  reset(); cmd("chassis id 1"); cmd("chassis readpos"); tick(30); assert(sent[2]==1 && sent[6]==0);
  cmd("arm read 5 version"); tick(30); assert(sent[6]==1 && strstr(output,"RX (7): 05 1F"));
  puts("PASS 02 prefixed diagnostics route only to the selected physical bus");
  reset(); cmd("chassis forward 100"); settle(); assert(triggers[2]==1 && !sent[6]);
  assert(models[1].dir==1 && models[2].dir==1 && models[3].dir==0 && models[4].dir==0);
  cmd("chassis square 100"); settle(); assert(triggers[2]==9);
  puts("PASS 03 chassis finite movement and explicit eight-segment square retained");
  reset(); cmd("arm lift up 1"); assert(!sent[6]); cmd("arm on"); cmd("arm lift up 1"); settle();
  assert(models[6].pulses==44 && models[6].dir==1 && !sent[2]);
  cmd("arm preset 6 1.5"); cmd("arm speed 6 8"); cmd("arm lift down"); settle();
  assert(models[6].pulses==66 && models[6].dir==0 && models[6].rpm==8);
  puts("PASS 04 arm unlock, measured direction, units, preset and per-axis speed retained");
  reset(); cmd("arm on"); cmd("arm plan rotate cw 3.6"); cmd("arm plan lift up 1"); assert(!sent[6]);
  cmd("arm run"); settle(); assert(triggers[6]==1 && moves[5]==1 && moves[6]==1 && moves[7]==1);
  puts("PASS 05 partial arm plan overwrites all pending axes before one bus-local sync");
  reset(); stale=1; cmd("chassis forward 100"); before=sent[6]; cmd("arm on"); cmd("arm move 5 0 32");
  assert(sent[6]==before && strstr(output,"Other subsystem busy")); cmd("!"); settle();
  assert(stops[2] && stops[6] && !disables); stale=0;
  cmd("arm lift up 1"); assert(!moves[6]);
  puts("PASS 06 cross-subsystem serialization and global ! stop retain lift enable and relock arm");
  reset(); cmd("arm on"); stale=1; cmd("arm rotate cw 3.6"); before=sent[2]; cmd("chassis forward 100"); assert(sent[2]==before);
  cmd("arm stop"); settle(); assert(stops[2] && stops[6]);
  puts("PASS 07 prefixed stop also cancels both subsystems");
  reset(); drop_id=2; cmd("chassis forward 100"); settle(); assert(MotorBench_HasFault() && ArmBench_HasFault() && stops[6]);
  before=moves[5]; cmd("arm on"); cmd("arm move 5 0 32"); assert(moves[5]==before);
  cmd("clearfault"); assert(MotorBench_HasFault()); drop_id=0; cmd("stopall"); settle(); cmd("clearfault");
  assert(!MotorBench_HasFault() && !ArmBench_HasFault()); cmd("arm lift up 1"); assert(!moves[6]);
  puts("PASS 08 chassis fault stops arm; failed stop cannot clear; explicit recovery never resumes");
  reset(); cmd("arm on"); stall_id=6; cmd("arm lift up 1"); settle(); assert(ArmBench_HasFault() && stops[2] && !disables);
  puts("PASS 09 arm stall propagates to chassis without disabling the lift");
  reset(); noise=1; cmd("chassis left 100"); settle(); cmd("arm on"); cmd("arm rail forward 1"); settle();
  assert(!MotorBench_HasFault() && !ArmBench_HasFault() && models[7].pulses==51);
  puts("PASS 10 unrelated packets on either bus cannot replace valid command replies");
  reset(); { uint8_t c='x'; for (i=0;i<600;++i) feed(&pc,&c,1); } tick(1); settle();
  assert(MotorBench_HasFault() && ArmBench_HasFault() && stops[2] && stops[6]);
  puts("PASS 11 console overflow requests global stop and latches faults");
  reset(); stale=1; cmd("chassis forward 100"); HAL_UART_ErrorCallback(&chassis); tick(1); settle(); assert(stops[6] && MotorBench_HasFault());
  reset(); cmd("arm on"); stale=1; cmd("arm lift up 1"); HAL_UART_ErrorCallback(&arm); tick(1); settle(); assert(stops[2] && ArmBench_HasFault());
  puts("PASS 12 active UART errors stop the whole robot and do not advance motion");
  reset(); { char big[220]; memset(big,'x',sizeof(big)); feed(&pc,(uint8_t *)big,sizeof(big)); feed(&pc,(uint8_t *)"\n",1); } tick(10);
  assert(!sent[2] && !sent[6] && strstr(output,"Line too long"));
  puts("PASS 13 overlong commands are discarded, never interpreted as truncated movement");
  assert(Mission_ParseCode("156+123+516+231",&parsed)); assert(parsed.temp_ring[1]==1 && parsed.temp_ring[5]==2 && parsed.temp_ring[6]==3);
  unchanged=parsed; assert(!Mission_ParseCode("156+123+514+231",&parsed) && !memcmp(&parsed,&unchanged,sizeof(parsed)));
  assert(!Mission_ParseCode("116+123+516+231",&parsed)); assert(!Mission_ParseCode("156+113+516+231",&parsed));
  assert(!Mission_ParseCode("156+123+516+230",&parsed)); assert(!Mission_ParseCode("156+123+516+231x",&parsed));
  puts("PASS 14 task parser validates four groups, distinct colors, ring permutations and same color sets atomically");
  reset(); connect_ports(0); cmd("mission home 2"); cmd("start"); for (i=0;i<40;++i) tick(1);
  assert(Mission_GetState()==MISSION_DONE && begins==33 && !sent[2] && !sent[6]);
  assert(history[0].op==GO_QR && history[0].home==2 && history[32].op==GO_HOME);
  for (i=0;i<3;++i) {
    assert(history[7+i].op==PLACE_ROUGH && history[10+i].op==RELOAD_ROUGH);
    assert(history[29+i].op==PLACE_TEMP && history[29+i].stack);
  }
  assert(history[29].color==5 && history[29].ring==2 && history[30].color==1 && history[30].ring==1 && history[31].ring==3);
  { MissionSnapshot s; Mission_GetSnapshot(&s); assert(s.grabs==12 && s.places==12 && s.home==2 && !strcmp(s.task_code,"156+123+516+231"));
    before=s.elapsed_ms; tick(10); Mission_GetSnapshot(&s); assert(s.elapsed_ms==before); }
  assert(!Mission_AcceptCode("156+123+516+231"));
  puts("PASS 15 mock services validate all 33 requests: batch barriers, matching-color stack and return to selected home");
  reset(); connect_ports(2); cmd("start"); tick(5); settle(); assert(Mission_GetState()==MISSION_FAULT && stops[2] && stops[6]);
  puts("PASS 16 QR service claiming done without a valid task causes fault, never advances to picking");
  reset(); connect_ports(1); cmd("start"); tick(500); assert(begins==1 && Mission_IsRunning()); ready_mask=0; tick(1); settle(); assert(cancels==1 && Mission_GetState()==MISSION_FAULT);
  puts("PASS 17 waiting feedback cannot advance; loss of service readiness cancels mission and stops both buses");
  reset(); connect_ports(1); cmd("start"); now+=14000U; tick(1); settle(); assert(Mission_GetState()==MISSION_FAULT && cancels==1);
  reset(); connect_ports(1); cmd("start"); now+=180000U; tick(1); settle(); assert(Mission_GetState()==MISSION_FAULT);
  puts("PASS 18 step and overall 180-second watchdogs cancel stalled mission");
  reset(); connect_ports(1); cmd("start"); before=sent[2]; cmd("chassis forward 100"); cmd("arm on"); assert(sent[2]==before && !sent[6]);
  assert(!Mission_RegisterPorts(NULL)); cmd("stopall"); settle(); assert(Mission_GetState()==MISSION_STOPPED && cancels==1);
  puts("PASS 19 mission excludes manual motion and service replacement; stop consumes the run");
  assert(App_Format(small,sizeof(small),"%u",123456U)==6 && !strcmp(small,"1234"));
  snprintf(expected,sizeof(expected),"%ld %lu %03lu %02X %d %s",LONG_MIN,ULONG_MAX,2UL,10U,-1,"ok");
  App_Format(actual,sizeof(actual),"%ld %lu %03lu %02X %d %s",LONG_MIN,ULONG_MAX,2UL,10U,-1,"ok"); assert(!strcmp(expected,actual));
  assert(App_Format(NULL,0,"%s","abc")==3);
  puts("PASS 20 compact integer formatter matches libc boundaries, padding and truncation without float runtime");
  reset(); assert(servo_timer.started==3U && !servo_timer.compare[0] && !servo_timer.compare[1]);
  cmd("servo status"); assert(strstr(output,"servo ready=1 fault=0 S1=0 us S2=0 us"));
  assert(!sent[2] && !sent[6]);
  puts("PASS 21 integrated servo initializes both channels at zero with no motor bus traffic");
  setup_servos(500U,1500U); cmd("servo S1 close"); cmd("servo s2 2");
  settle_servo();
  cmd("servo both 500 2500"); cmd("servo s1 1500");
  assert(servo_timer.compare[0]==CLAW_CLOSE && servo_timer.compare[1]==1500);
  puts("PASS 22 measured S1 close and S2 state commands route correctly; removed numeric/dual commands cannot move outputs");
  stale=1; cmd("chassis forward 100"); cmd("servo s1 open"); cmd("servo status");
  assert(servo_timer.compare[0]==CLAW_CLOSE && servo_timer.compare[1]==1500 && strstr(output,"Servo change blocked"));
  cmd("!"); settle(); assert(!servo_timer.compare[0] && !servo_timer.compare[1]);
  reset(); cmd("arm on"); stale=1; cmd("arm rotate cw 3.6"); cmd("servo s1 close");
  assert(!servo_timer.compare[0] && !servo_timer.compare[1]); cmd("stopall"); settle();
  puts("PASS 23 chassis/arm busy interlock rejects servo changes; immediate stop clears both PWM targets");
  { static const char *aliases[]={"stop","stopall","chassis stop","arm stop","servo stop","servo STOP"};
    for (i=0;i<sizeof(aliases)/sizeof(aliases[0]);++i) {
      reset(); setup_servos(500U,1500U); cmd("servo s1 close"); cmd(aliases[i]); settle();
      assert(!servo_timer.compare[0] && !servo_timer.compare[1] && stops[2] && stops[6]);
    }
  }
  puts("PASS 24 all documented global/prefixed stop aliases stop both motor buses and clear servo PWM");
  reset(); servo_timer.Init.Prescaler=15U; Servo_Init(&servo_timer); tick(1); settle(); assert(Servo_HasFault());
  cmd("clearfault"); cmd("chassis forward 100"); cmd("servo s1 1500");
  assert(!moves[1] && !servo_timer.compare[0] && Servo_HasFault() && strstr(output,"Servo init fault"));
  reset(); setup_servos(500U,1500U); stall_id=3; cmd("chassis forward 100"); settle();
  assert(MotorBench_HasFault() && !servo_timer.compare[0] && !servo_timer.compare[1]);
  puts("PASS 25 servo init fault cannot be cleared into motion; a motor fault also zeros both servo channels");
  reset(); connect_ports(1); cmd("start"); cmd("servo s1 close");
  assert(Mission_IsRunning() && !servo_timer.compare[0] && !servo_timer.compare[1]);
  cmd("servo stop"); settle(); assert(Mission_GetState()==MISSION_STOPPED && cancels==1);
  puts("PASS 26 active mission excludes manual servo commands; servo stop cancels mission without resuming outputs");
  reset(); setup_servos(500U,1500U); cmd("servo claw open"); assert(servo_timer.compare[0]==500U);
  cmd("servo tray 2"); cmd("servo speed s1 27"); cmd("servo claw open"); cmd("servo tray 3");
  cmd("servo claw close");
  assert(Servo_IsBusy()); settle_servo(); assert(servo_timer.compare[0]==CLAW_CLOSE && servo_timer.compare[1]==2080U && !sent[2] && !sent[6]);
  puts("PASS 27 semantic claw/tray commands and independent speed changes run concurrently without motor bus traffic");
  reset(); setup_servos(500U,1500U); cmd("servo s1 close"); cmd("servo s2 3");
  cmd("chassis forward 100"); cmd("arm on"); cmd("arm read 5 version"); connect_ports(1); cmd("start");
  assert(!sent[2] && !sent[6] && !Mission_IsRunning() && strstr(output,"Servo ramp busy"));
  cmd("servo status"); settle_servo(); cmd("chassis forward 100"); settle(); assert(moves[1]==1U);
  puts("PASS 28 pending servo ramp excludes chassis/arm operations and mission start; completion releases interlock");
  reset(); setup_servos(500U,1500U); cmd("servo s1 close"); tick(100); cmd("servo speed s1 10"); cmd("servo claw open");
  settle_servo(); assert(servo_timer.compare[0]==500U);
  cmd("servo claw close"); cmd("!"); settle(); tick(2000);
  assert(!Servo_IsBusy() && !servo_timer.compare[0] && !servo_timer.compare[1] && stops[2] && stops[6]);
  puts("PASS 29 retarget/speed edits remain available during a ramp; urgent stop cancels queued output permanently");
  reset(); realistic_duration=monitor_running=1U; cmd("chassis speed 10"); setup_servos(500U,1500U); cmd("chassis forward 1000");
  for (i=0;i<26U;++i) { tick(1000U); cmd("status"); }
  assert(!MotorBench_IsBusy() && !MotorBench_HasFault() && !ArmBench_HasFault());
  assert(triggers[2]==1U && !sent[6] && !running_writes && !stops[2] && !stops[6]);
  assert(servo_timer.compare[0]==500U && servo_timer.compare[1]==1500U && !Servo_IsBusy());
  for (i=1;i<=4;++i) assert(moves[i]==1U && models[i].pulses==12732U && models[i].rpm==10U);
  puts("PASS 30 full C5.2 loop with live holding PWM and repeated status queries preserves one uninterrupted 24-second nominal chassis command");
  reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); cmd("route1 next"); assert(!sent[2] && !sent[6]); raw_station();
  assert(triggers[2]==4U && MotorBench_IsBusy() && !Mission_IsRunning());
  before=sent[2]; cmd("route1 status"); cmd("servo claw open"); cmd("arm on"); cmd("chassis forward 100"); cmd("start");
  tick(20000); assert(sent[2]==before && arm_n==0U && servo_timer.compare[0]==CLAW_CLOSE);
  cmd("route1 next"); wait_text("ROUTE1 rough"); assert(triggers[2]==6U);
  cmd("route1 next"); wait_text("ROUTE1 temp"); assert(triggers[2]==11U);
  settle(); assert(triggers[2]==17U && !MotorBench_HasFault());
  assert(arm_n==18U && servo_timer.compare[0]==500U && servo_timer.compare[1]==1500U);
  {
    const unsigned ids[]={5,6,6,5, 6,6,5,6,6,6,6,5, 6,6,5,6,6,5};
    const unsigned pulses[]={
      1600,3288,3288,1600,
      1096,1096,1600,6926,6926,6926,6926,1600,
      1096,1096,1600,6926,6926,1600};
    const unsigned closed[]={1,0,1,1, 0,1,1,1,0,0,1,1, 0,1,1,1,0,0};
    const unsigned directions[]={0,0,1,1, 0,1,0,0,1,0,1,1, 0,1,0,0,1,1};
    for(i=0;i<arm_n;++i) {
      assert(arm_ids[i]==ids[i] && arm_pulses[i]==pulses[i]);
      assert(arm_dirs[i]==directions[i]);
      assert(arm_claw[i]==(closed[i]?CLAW_CLOSE:500U) && arm_station[i]==(i<4U?4U:i<12U?6U:11U));
    }
  }
  assert(models[5].pos==0 && models[6].pos==0 && !moves[7]);
  puts("PASS 31 route1 executes 18 arm moves and releases at the top with correct claw state and stationary chassis; only raw requires next");
  reset(); setup_servos(500U,1500U); raw_station(); cmd("route1 stop"); settle();
  before=triggers[2]; cmd("route1 next"); tick(2000); assert(triggers[2]==before && stops[2] && stops[6] && !servo_timer.compare[0]);
  puts("PASS 32 route1 stop requests global cancellation and PWM clearing; next cannot resume the canceled course");
  reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); raw_station(); drop_id=2; cmd("route1 next"); settle();
  assert(MotorBench_HasFault() && ArmBench_HasFault() && triggers[2]==4U && stops[6]);
  cmd("route1 next"); assert(triggers[2]==4U);
  puts("PASS 33 route1 resume preflight failures propagate globally without starting the next leg");
  reset(); prepare_reference(); cmd("route1 start"); wait_text("ROUTE1: check QR"); cmd("route1 next"); now+=60001U; tick(1); settle();
  assert(MotorBench_HasFault() && ArmBench_HasFault() && triggers[2]==2U && stops[6]);
  puts("PASS 34 route1 QR timeout cannot be bypassed by next and stops both buses");
  reset(); raw_station(); HAL_UART_ErrorCallback(&chassis); tick(1); settle();
  assert(MotorBench_HasFault() && stops[2] && stops[6]);
  puts("PASS 35 UART failure during a manual station wait still causes the global stop");
  reset(); prepare_reference(); cmd("route1 start");
  { unsigned n=10000U; while (n-- && triggers[2]<2U) tick(1); assert(triggers[2]==2U); }
  stale=1U; scan_route(); tick(1200); before=hmi_blank;
  { char task[16]; assert(QrBridge_GetCode(task,NULL,NULL)); }
  cmd("route1 next"); assert(models[1].moving && triggers[2]==2U);
  stale=0U; wait_text("ROUTE1 raw"); assert(triggers[2]==4U && hmi_blank==before && !arm_n);
  puts("PASS 36 full loop preserves the first approach scan on the display, waits for arrival, and reaches raw without rescanning");
  reset(); prepare_reference(); cmd("route1 start");
  { unsigned n=10000U; while (n-- && triggers[2]<2U) tick(1); assert(triggers[2]==2U); }
  stale=1U; scan_route(); cmd("route1 stop"); settle(); before=triggers[2];
  cmd("route1 next"); tick(2000); assert(triggers[2]==before && stops[2] && stops[6]);
  puts("PASS 37 retaining an approach QR never resumes movement after a global route cancellation");
  reset(); cmd("servo speed s1 1"); cmd("servo speed s2 1"); cmd("servo claw open");
  cmd("servo s1 1574"); cmd("servo tray 2"); cmd("servo both 1500 1500"); cmd("servo s1 close");
  tick(2000); assert(!servo_timer.compare[0] && !servo_timer.compare[1] && !Servo_IsBusy() && !sent[2] && !sent[6]);
  assert(strstr(output,"Invalid servo") && strstr(output,"Servo start unknown"));
  puts("PASS 38 measured claw commands still reject an unknown initial output reference");
  reset(); cmd("servo setup s1 500"); cmd("servo setup s1 500 yes"); cmd("servo setup s1 751 unloaded"); cmd("servo setup s2 2501 unloaded");
  assert(!servo_timer.compare[0] && !servo_timer.compare[1]);
  cmd("servo setup s1 500 unloaded"); assert(servo_timer.compare[0]==500U && !servo_timer.compare[1]);
  { char text[48]; snprintf(text,sizeof(text),"servo setup s1 %u unloaded",(unsigned)CLAW_CLOSE); cmd(text); } assert(servo_timer.compare[0]==500U && !Servo_IsBusy());
  puts("PASS 39 integrated commissioning is explicit and one-time per output-off interval; it cannot overwrite an active target instantly");
  cmd("servo speed s1 1"); cmd("servo claw close"); tick(1000);
  assert(servo_timer.compare[0]>=507U && servo_timer.compare[0]<=508U && Servo_IsBusy());
  cmd("!"); settle(); cmd("clearfault"); cmd("servo claw open"); tick(2000);
  assert(!servo_timer.compare[0] && !servo_timer.compare[1] && !Servo_IsBusy());
  puts("PASS 40 configured claw ramps at 1deg/s from an established reference; stop/clearfault cannot silently reattach it");
  reset(); stale=1U; cmd("chassis forward 100"); cmd("servo setup s1 500 unloaded");
  assert(!servo_timer.compare[0] && strstr(output,"Servo change blocked")); cmd("!"); settle();
  reset(); connect_ports(1); cmd("start"); cmd("servo setup s1 500 unloaded"); assert(!servo_timer.compare[0]); cmd("stopall"); settle();
  reset(); raw_station(); cmd("servo setup s1 500 unloaded"); assert(!servo_timer.compare[0] && MotorBench_IsBusy());
  puts("PASS 41 unloaded setup remains subject to chassis, full-mission and route1 interlocks");
  reset(); setup_servos(500U,1500U); cmd("servo speed s1 27"); cmd("servo s1 close"); tick(100);
  cmd("servo setup s1 751 unloaded"); cmd("servo s1 750"); cmd("servo s1 1500");
  cmd("servo map s1 270 500 2500"); cmd("servo claw set close 900"); cmd("servo both 500 2000");
  assert(!Servo_Set(751U,2000U) && !Servo_Set(CLAW_CLOSE,2501U)); settle_servo();
  assert(servo_timer.compare[0]==CLAW_CLOSE && servo_timer.compare[1]==1500U && !Servo_HasFault());
  cmd("servo s1 open"); settle_servo(); assert(servo_timer.compare[0]==500U);
  puts("PASS 42 rejected S1 pulse/bypass inputs leave the bounded active ramp and S2 untouched; canonical open reaches exactly 500us");
  reset(); setup_servos(500U,850U); cmd("servo speed s2 27"); cmd("servo s2 3"); tick(100);
  assert(Servo_IsBusy() && servo_timer.compare[1]>850U && servo_timer.compare[1]<2080U);
  cmd("chassis forward 100"); cmd("arm on"); cmd("start"); assert(!sent[2] && !sent[6] && !Mission_IsRunning());
  cmd("servo s2 1500"); cmd("servo tray 150"); cmd("servo map s2 270 500 2500"); cmd("servo s2 4");
  assert(!Servo_Set(CLAW_CLOSE,1450U)); settle_servo(); assert(servo_timer.compare[0]==500U && servo_timer.compare[1]==2080U && !Servo_HasFault());
  cmd("servo tray 2"); settle_servo(); assert(servo_timer.compare[1]==1500U); cmd("servo s2 1"); settle_servo(); assert(servo_timer.compare[1]==850U);
  cmd("servo stop"); settle(); cmd("clearfault"); cmd("servo s2 2"); assert(!servo_timer.compare[1]);
  puts("PASS 43 finite tray states retain whole-vehicle interlocks, reject obsolete inputs during ramps and require a new reference after stop");
  reset(); cmd("servo pulse s1 650"); cmd("servo pulse s2 1700"); assert(!servo_timer.compare[0] && !servo_timer.compare[1]);
  setup_servos(500U,1500U); cmd("servo speed s1 1"); cmd("servo speed s2 1"); cmd("servo pulse s1 650"); cmd("servo pulse s2 1700"); tick(1000);
  assert(servo_timer.compare[0]>500U && servo_timer.compare[0]<650U && servo_timer.compare[1]>1500U && servo_timer.compare[1]<1700U);
  cmd("chassis forward 100"); cmd("arm on"); cmd("start"); assert(!sent[2] && !sent[6] && !Mission_IsRunning());
  cmd("servo pulse s1 851"); cmd("servo pulse s2 2501"); cmd("servo speed s1 30"); cmd("servo speed s2 30"); settle_servo();
  assert(servo_timer.compare[0]==650U && servo_timer.compare[1]==1700U && !Servo_HasFault());
  stale=1U; cmd("chassis forward 100"); cmd("servo pulse s1 600"); cmd("servo pulse s2 1800");
  assert(servo_timer.compare[0]==650U && servo_timer.compare[1]==1700U); cmd("!"); settle();
  cmd("clearfault"); cmd("servo pulse s1 600"); cmd("servo pulse s2 1800"); assert(!servo_timer.compare[0] && !servo_timer.compare[1]);
  puts("PASS 44 raw pulse debugging shares startup protection, whole-vehicle interlocks, speed ramps, boundaries and stop cancellation");
  {
    static const char *commands[]={"chassis forward 100","chassis backward 100","chassis left 100","chassis right 100","chassis turn left 30","chassis turn right 30"};
    static const uint8_t directions[6][4]={{1,1,0,0},{0,0,1,1},{0,1,1,0},{1,0,0,1},{1,1,1,1},{0,0,0,0}};
    for (i=0;i<6U;++i) {
      reset(); cmd(commands[i]); settle();
      assert(triggers[2]==1U && !sent[6] && !MotorBench_HasFault());
      for (unsigned j=1U;j<=4U;++j) assert(models[j].dir==directions[i][j-1U] && models[j].pulses==(i<4U?1273U:1350U));
      assert(strstr(output,"M1 delta=") && strstr(output,"M4 delta="));
    }
  }
  puts("PASS 45 prefixed chassis commands retain all six direction frames, distances and wheel travel diagnostics");
  {
    static const char *commands[]={"arm rotate cw 1.8","arm rotate ccw 1.8","arm lift up 2","arm lift down 2","arm rail forward 2","arm rail backward 2"};
    static const unsigned ids[]={5,5,6,6,7,7},directions[]={0,1,1,0,1,0},pulses[]={16,16,88,88,102,102};
    for (i=0;i<6U;++i) {
      reset(); cmd(commands[i]); assert(!sent[6]); cmd("arm on"); cmd(commands[i]); settle();
      assert(!sent[2] && models[ids[i]].dir==directions[i] && models[ids[i]].pulses==pulses[i]);
    }
  }
  puts("PASS 46 prefixed arm commands retain locking, measured units, direction and finite motion");
  reset(); cmd("chassis speed 20"); cmd("chassis speed 301"); cmd("chassis forward 100"); settle();
  assert(models[1].rpm==20U); cmd("chassis turn right 30"); settle(); assert(models[1].rpm==20U);
  cmd("arm lift up 2"); assert(!sent[6]);
  puts("PASS 47 prefixed speed retains shared chassis speed semantics and rejects 301RPM");
  reset(); setup_servos(500U,1500U); cmd("servo s1 close"); settle_servo(); assert(servo_timer.compare[0]==CLAW_CLOSE);
  cmd("servo s2 3"); settle_servo(); assert(servo_timer.compare[1]==2080U);
  cmd("servo s1 1500"); cmd("servo s2 4"); assert(servo_timer.compare[0]==CLAW_CLOSE && servo_timer.compare[1]==2080U);
  puts("PASS 48 prefixed servo commands retain presets, bounds, reference requirements and ramps");
  {
    static const char *commands[]={"servo StOp"," servo\tSTOP  ","arm\tOFF  "," STOPALL ","chassis StOp"," ARM OFF ","route1\tstop "};
    for (i=0;i<sizeof(commands)/sizeof(commands[0]);++i) {
      reset(); setup_servos(500U,1500U); stale=1U; cmd("chassis forward 100");
      cmd(commands[i]); settle();
      assert(stops[2] && stops[6] && !servo_timer.compare[0] && !servo_timer.compare[1]);
    }
  }
  puts("PASS 49 mixed case, tabs and outer spaces normalize before global stop and busy checks");
  reset(); cmd("  CHASSIS\t SPEED 12  "); cmd(" CHASSIS\t FORWARD 100  "); settle();
  assert(models[1].rpm==12U && models[1].dir==1U && strstr(output,"M1 delta=-"));
  assert(strstr(output,"M2 delta=-") && strstr(output,"M3 delta=") && strstr(output,"M4 delta="));
  { char big[161]; memset(big,'x',sizeof(big)); memcpy(big,"chassis forward ",16U); big[160]='\0';
    before=sent[2]; Competition_Command(big); assert(sent[2]==before && strstr(output,"Command too long")); }
  puts("PASS 50 normalized movement keeps signed diagnostics; overlong API input cannot become truncated motion");
  reset(); stale=1U; cmd("chassis forward 100"); before=sent[6]; cmd("arm on"); cmd("arm lift up 2"); cmd("servo s1 close");
  assert(sent[6]==before && !servo_timer.compare[0]); cmd("stop"); settle();
  reset(); setup_servos(500U,1500U); cmd("servo s1 close"); cmd("chassis forward 100"); cmd("arm on"); assert(!sent[2] && !sent[6]);
  cmd("stop"); settle(); reset(); connect_ports(1); cmd("start"); cmd("chassis forward 100"); cmd("arm on"); assert(!sent[2] && !sent[6]);
  puts("PASS 51 prefixed commands retain cross-subsystem, servo and mission interlocks");
  reset();
  {
    static const char *bad[]={"chassis tx 01 FD 00 03 E8 0A 00 01 86 A0 02 00 6B", "chassis tx 00 FF 66 6B",
      "chassis tx 01 F3 AB 01 00 6B", "chassis tx 01 AE 4B 01 02 6B", "chassis tx 01 FE 98 00 6B",
      "chassis tx 00 36 6B", "chassis tx 01 36 00", "chassis tx 01 36 6B 6B", "chassis tx 01 42 00 6B"};
    for (i=0;i<sizeof(bad)/sizeof(bad[0]);++i) cmd(bad[i]);
    assert(!sent[2] && !sent[6] && !models[1].moving);
  }
  cmd("chassis tx 01 36 6B"); tick(30); assert(sent[2]==1U && strstr(output,"RX (8): 01 36"));
  puts("PASS 52 raw chassis TX allows exact read queries only; motion, writes, broadcast, malformed and extended frames send nothing");
  reset(); cmd("on"); cmd("arm lift up 2"); assert(!sent[2] && !sent[6]);
  setup_servos(500U,1500U); cmd("arm on");
  {
    static const char *removed[]={"f 100","b 100","l 100","r 100","tl 30","tr 30","v 20",
      "on","off","cw 3.6","ccw 3.6","up 2","down 2","out 2","in 2","s1 close","s2 3"," F\t100 "," OFF "};
    for (i=0;i<sizeof(removed)/sizeof(removed[0]);++i) cmd(removed[i]);
    assert(!sent[2] && !sent[6] && !Servo_IsBusy() && servo_timer.compare[0]==500U && servo_timer.compare[1]==1500U);
    cmd("arm lift up 2"); settle(); assert(models[6].pulses==88U);
  }
  puts("PASS 53 all removed direct aliases reject without motion, target or arming changes; prefixed arm remains armed");
  reset(); cmd("arm on"); cmd("arm rail backward 10 60"); settle();
  assert(models[7].pulses==509U && models[7].dir==0U && models[7].rpm==60U);
  assert(strstr(output,"M7 backward: 10.000 mm -> 509 pulses;") && strstr(output,"DONE: arm encoders verified"));
  cmd("arm rail forward 10 60"); settle(); assert(models[7].pulses==509U && models[7].dir==1U && models[7].pos==0);
  cmd("arm rail forward 100"); settle(); assert(models[7].pulses==5093U && models[7].dir==1U && models[7].rpm==150U);
  before=sent[6]; cmd("arm rail forward 100.001"); cmd("arm move 7 0 5094"); assert(sent[6]==before);
  cmd("arm plan rail backward 10 60"); before=sent[6]; cmd("arm run"); settle();
  assert(sent[6]>before && triggers[6]==1U && models[7].pulses==509U && models[7].dir==0U);
  assert(!sent[2] && !MotorBench_HasFault() && !ArmBench_HasFault());
  puts("PASS 54 user M7 backward 10mm/60RPM emits 509 pulses DIR0 at the pinion ratio; forward is DIR1; 100mm, cap, plan and other bus isolation remain valid");
  reset(); assert(!sent[2] && !sent[6]); cmd("chassis forward 100"); settle();
  for(i=1;i<=4U;++i) assert(models[i].rpm==150U);
  cmd("chassis turn left 30"); settle(); for(i=1;i<=4U;++i) assert(models[i].rpm==150U);
  cmd("arm on"); cmd("arm rotate cw 3.6"); settle(); assert(models[5].rpm==50U);
  cmd("arm lift up 2"); settle(); assert(models[6].rpm==150U);
  cmd("arm rail forward 2"); settle(); assert(models[7].rpm==150U && models[7].pulses==102U && models[7].dir==1U);
  cmd("arm speed all 10"); cmd("arm speed defaults"); cmd("arm plan lift up 2"); cmd("arm run"); settle();
  assert(models[5].rpm==50U && models[6].rpm==150U && models[7].rpm==150U);
  reset(); cmd("chassis forward 100"); settle(); cmd("arm on"); cmd("arm rail backward 2"); settle();
  assert(models[1].rpm==150U && models[7].rpm==150U);
  puts("PASS 55 power-on remains motionless; chassis defaults to 150RPM; arm axes use 50/150/150RPM");
  reset(); cmd("chassis speed 300"); cmd("chassis speed 301"); cmd("chassis forward 100"); settle();
  for(i=1;i<=4U;++i) assert(models[i].rpm==300U);
  cmd("chassis turn right 30"); settle(); for(i=1;i<=4U;++i) assert(models[i].rpm==300U);
  cmd("arm on"); cmd("arm speed all 150"); cmd("arm speed all 151");
  cmd("arm speed 6 200"); cmd("arm speed 7 200"); cmd("arm speed 6 201"); cmd("arm speed 7 201");
  cmd("arm plan rotate cw 3.6"); cmd("arm plan lift up 2"); cmd("arm plan rail forward 2"); cmd("arm run"); settle();
  assert(models[5].rpm==150U && models[6].rpm==200U && models[7].rpm==200U);
  before=sent[6]; cmd("arm move 5 0 32 151"); cmd("arm lift up 2 201"); cmd("arm plan 7 1 32 201");
  cmd("arm run"); assert(sent[6]==before);
  cmd("arm move 7 0 32 200"); settle(); assert(models[7].rpm==200U);
  puts("PASS 56 chassis and arm axes enforce their 300/150/200/200RPM limits in command frames");
  reset(); cmd("arm convert 7 mm 62.83"); cmd("arm convert 7 pulses 3200");
  assert(!sent[6] && !sent[2] && strstr(output,"62.830 mm -> 3200 pulses") && strstr(output,"3200 pulses = 62.830 mm"));
  cmd("arm on"); cmd("arm rail forward 62.83"); settle();
  assert(models[7].pulses==3200U && models[7].rpm==150U && models[7].dir==1U);
  cmd("arm rail backward 62.83"); settle(); assert(models[7].pulses==3200U && models[7].dir==0U && models[7].pos==0);
  cmd("arm rail forward"); settle(); assert(models[7].pulses==2547U);
  cmd("arm plan rail backward 62.83"); cmd("arm run"); settle(); assert(models[7].pulses==3200U && models[7].dir==0U);
  before=sent[6]; cmd("arm rail forward 100.001"); cmd("arm plan rail backward 100.001"); cmd("arm move 7 0 5094");
  assert(sent[6]==before && !sent[2] && !ArmBench_HasFault());
  puts("PASS 57 M7 one pinion revolution is exactly 62.830mm/3200 pulses through preview, both directions and plans; 50mm preset and 100mm bounds retain nearest-pulse conversion");
  reset(); raw_station(); cmd("route1 next"); settle();
  assert(!arm_n && MotorBench_HasFault() && !servo_timer.compare[0] && triggers[2]==4U);
  puts("PASS 58 missing S1 reference aborts at raw without moving the arm or leaving the station");
  reset(); setup_servos(500U,1500U); raw_station(); cmd("route1 next"); settle();
  assert(!arm_n && MotorBench_HasFault() && triggers[2]==4U);
  puts("PASS 59 locked arm command rejection faults the route rather than pretending completion");
  { unsigned cut;
    for(cut=0U;cut<18U;++cut) {
      reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); raw_station(); cmd("route1 next");
      { unsigned limit=40000U; while(limit-- && arm_n<=cut && !MotorBench_HasFault()) tick(1); assert(arm_n==cut+1U); }
      cmd("!"); settle(); before=arm_n; tick(5000); cmd("route1 next"); tick(5000);
      assert(arm_n==before && triggers[2]==(cut<4U?4U:cut<12U?6U:11U) && !servo_timer.compare[0] && !Servo_IsBusy());
    }
  }
  puts("PASS 60 urgent stop during each of 18 arm moves cancels later steps and next never resumes");
  reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); raw_station(); drop_id=6U; cmd("route1 next"); tick(5000); settle();
  assert(MotorBench_HasFault() && ArmBench_HasFault() && triggers[2]==4U && !servo_timer.compare[0]);
  puts("PASS 61 missing arm motor feedback aborts all buses and never releases the chassis");
  { unsigned release;
    for(release=0U;release<2U;++release) {
      reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); raw_station(); cmd("route1 next");
      { unsigned limit=20000U; while(limit-- && !(arm_n==(release?4U:2U) && !ArmBench_IsBusy() && Servo_IsBusy() && servo_timer.compare[0]>500U && servo_timer.compare[0]<CLAW_CLOSE)) tick(1); assert(limit); }
      before=arm_n; cmd("route1 next"); assert(arm_n==before && triggers[2]==4U);
      cmd("stopall"); settle(); tick(5000); assert(arm_n==before && triggers[2]==4U && !servo_timer.compare[0]);
    }
  }
  puts("PASS 62 next cannot skip either claw-only wait; stop cancels both close and release waits");
  reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); raw_station(); cmd("route1 next");
  { unsigned limit=20000U; while(limit-- && !(arm_n==2U && !ArmBench_IsBusy() && Servo_IsBusy())) tick(1); assert(limit); }
  before=arm_n; now+=120001U; tick(1); settle(); assert(MotorBench_HasFault() && arm_n==before && triggers[2]==4U && !servo_timer.compare[0]);
  puts("PASS 63 coordinator timeout while waiting for the claw aborts without starting lift-up or leaving station");
  reset(); cmd("route"); cmd("route show"); cmd("route show 1"); cmd("route export");
  assert(!sent[2] && !sent[6] && !servo_timer.compare[0] && !servo_timer.compare[1]);
  assert(strstr(output,"route 1 route1") && strstr(output,"21 steps") && strstr(output,"segment 0 to_scan") &&
         strstr(output,"group 1 raw_load") && strstr(output,"group 2 rough_transfer") && strstr(output,"group 3 temp_unload") &&
         strstr(output,"#define RP_ARM_RAW 75000U") && strstr(output,"#define RP_ARM_TEMP 158000U") &&
         strstr(output,"#define RP_ARM_DROP 0U") && strstr(output,"#define RP_ARM_TRAY_PICK 25000U"));
  puts("PASS 64 route directory, fully expanded preview and export are read-only and include shared group values");
  reset(); cmd("route set center 135"); cmd("route set arm_rough_place 170.5"); cmd("route show 1"); cmd("route export");
  assert(strstr(output,"center=135 mm") && strstr(output,"arm_rough_place=170.500 mm") && strstr(output,"#define RP_CENTER 135U") && strstr(output,"#define RP_ARM_ROUGH_PLACE 170500U") && !sent[2] && !sent[6]);
  { RouteStep e; RouteWorkStep w; assert(RouteLibrary_Step(1,0,&e) && e.value==135U); assert(RouteLibrary_Work(2,4,&w) && w.amount==170500U); assert(RouteLibrary_Work(2,6,&w) && w.amount==170500U); }
  cmd("route reset"); { RouteStep e; RouteWorkStep w; assert(RouteLibrary_Step(1,0,&e) && e.value==125U); assert(RouteLibrary_Work(2,4,&w) && w.amount==158000U); }
  puts("PASS 65 named RAM values support arm decimals, export integer macros and update both lowering/return references atomically");
  reset();
  { const char *bad[]={"route set center 9","route set center 3001","route set center -1","route set center 1.5","route set center 4294967297",
      "route set rough_turn 361","route set arm_turn 180.001","route set arm_pick 200.001","route set arm_rough_place 0",
      "route set arm_pick .5","route set arm_pick 1.0001","route set arm_rpm 151","route set drive_rpm 301","route set missing 10","route set center 135 extra"};
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);++i) cmd(bad[i]); }
  { RouteStep e; RouteWorkStep w; assert(RouteLibrary_Step(1,0,&e) && e.value==125U); assert(RouteLibrary_Work(1,0,&w) && w.amount==180000U); }
  assert(!sent[2] && !sent[6] && !Servo_IsBusy());
  puts("PASS 66 invalid units, overflow, speed/travel bounds, unknown parameters and trailing tokens leave route values unchanged");
  reset(); cmd("route set center 135"); cmd("route start 255"); cmd("route start 0"); cmd("route start 1 extra"); cmd("route start 3"); cmd("route start 4"); cmd("route start 5"); cmd("route start 6");
  assert(!sent[2] && !sent[6] && !MotorBench_IsBusy());
  assert(!RouteLibrary_Validate(3) && !RouteLibrary_Validate(4) && !RouteLibrary_Validate(5) && !RouteLibrary_Validate(6));
  puts("PASS 67 missing paths, invalid segment indices, wrong parameter types, unavailable groups and incompatible joins reject before any motor traffic");
  reset(); cmd("route set center 135"); cmd("route set scan_go 1000"); cmd("route set drive_rpm 42"); cmd("route start 7"); wait_text("ROUTE7: check QR");
  assert(triggers[2]==2U && models[1].rpm==42U && models[1].pulses==12732U);
  cmd("route set center 145"); cmd("route reset"); cmd("route start 1"); cmd("route next");
  { RouteStep e; assert(RouteLibrary_Step(1,0,&e) && e.value==135U); }
  scan_route(); settle(); assert(!MotorBench_HasFault() && triggers[2]==2U && !sent[6]);
  puts("PASS 68 a second test-only path reuses the scan segment and same executor, inherits edits, and completes on a terminal QR event");
  reset(); cmd("route set center 135"); stale=1U; cmd("chassis forward 100"); cmd("route set center 145"); cmd("route reset");
  { RouteStep e; assert(RouteLibrary_Step(1,0,&e) && e.value==135U); } cmd("!"); settle();
  cmd("route set center 145"); { RouteStep e; assert(RouteLibrary_Step(1,0,&e) && e.value==145U); }
  reset(); { RouteStep e; assert(RouteLibrary_Step(1,0,&e) && e.value==125U); }
  puts("PASS 69 edits/default reset are locked during a single chassis move, allowed only after stopping, and MCU init restores defaults");
  reset(); setup_servos(500U,1500U); cmd("servo s1 close"); cmd("route set center 145"); cmd("route reset");
  { RouteStep e; assert(RouteLibrary_Step(1,0,&e) && e.value==125U); } cmd("!"); settle();
  reset(); connect_ports(1); cmd("start"); cmd("route set center 145"); cmd("route reset");
  { RouteStep e; assert(RouteLibrary_Step(1,0,&e) && e.value==125U); } cmd("!"); settle();
  reset(); drop_id=1U; cmd("chassis forward 100"); settle(); cmd("route set center 145"); cmd("route reset");
  { RouteStep e; assert(RouteLibrary_Step(1,0,&e) && e.value==125U); }
  puts("PASS 70 servo ramps, mission activity and latched faults all prevent route parameter mutation");
  reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); cmd("route set arm_turn 90"); cmd("route set arm_pick 170.5"); cmd("route set arm_rough_place 170.5"); cmd("route set arm_drop 15.5");
  cmd("route set arm_raw 99.5"); cmd("route set arm_temp 175.5"); cmd("route set arm_tray_pick 26.5");
  cmd("route set arm_rpm 12"); cmd("route set turn_rpm 24"); cmd("route set drive_rpm 42");
  prepare_reference(); cmd("route start 1"); wait_text("ROUTE1: check QR"); scan_route(); wait_text("ROUTE1 raw"); cmd("route next"); wait_text("ROUTE1 rough");
  cmd("route next"); cmd("route set arm_pick 100"); wait_text("ROUTE1 temp"); settle();
  assert(triggers[2]==17U && arm_n==22U && !moves[7] && models[5].pos==0 && models[6].pos==0 && servo_timer.compare[0]==500U);
  assert(arm_pulses[0]==800U && arm_pulses[3]==800U && arm_pulses[1]==4362U && arm_pulses[2]==4362U &&
         arm_pulses[4]==679U && arm_pulses[5]==679U && arm_pulses[6]==1162U && arm_pulses[7]==1162U &&
         arm_pulses[9]==7474U && arm_pulses[12]==7474U && arm_pulses[14]==679U && arm_pulses[15]==679U &&
         arm_pulses[16]==1162U && arm_pulses[17]==1162U &&
         arm_pulses[19]==7693U && arm_pulses[20]==7693U);
  assert(models[5].rpm==12U && models[6].rpm==12U && models[1].rpm==42U);
  { RouteWorkStep w; assert(RouteLibrary_Work(2,4,&w) && w.amount==170500U); }
  puts("PASS 71 full generic path executes edited decimal work points and speed profiles, forbids edits/skip during work and returns net arm pose");
  reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); prepare_reference(); cmd("route start 1"); wait_text("ROUTE1: check QR"); scan_route(); wait_text("ROUTE1 raw");
  cmd("route next"); wait_text("ROUTE1 rough"); cmd("route stop"); settle(); before=arm_n; cmd("route next"); tick(3000);
  assert(arm_n==before && triggers[2]==6U && !servo_timer.compare[0]);
  puts("PASS 72 generic stop alias cancels group execution and generic next cannot resume a canceled route");
  reset(); tick(310);
  assert(strstr(output,"@state 0 0 idle idle 0 0 0 0 0 0 0 0") && !sent[2] && !sent[6]);
  setup_servos(500U,1500U); cmd("arm on"); cmd("status");
  assert(strstr(output,"@state 1 0 idle idle 0 0 0 0 0 500 1500 0"));
  cmd("arm lift down 10"); settle();
  {
    const char *axes=strstr(output,"@axes ");
    unsigned mask,r5,r6,r7,p5,p6,p7;
    assert(axes && sscanf(axes,"@axes %u %u %u %u %u %u %u",&mask,&r5,&r6,&r7,&p5,&p6,&p7)==7);
    assert(mask==2U && r6==0U && p6>=999U && p6<=1000U);
  }
  stall_id=6U; cmd("arm lift up 10"); settle(); cmd("status");
  assert(strstr(output,"@state 0 1 fault fault"));
  used=0U; output[0]='\0'; cmd("clearfault"); cmd("status");
  assert(strstr(output,"Cleared; arm locked; no resume.") && strstr(output,"@state 0 0 idle idle"));
  puts("PASS 73 live status reports arm interlock, global fault, PWM and encoder axis feedback");
  reset(); cmd("route1 start"); cmd("chassis route1"); cmd("route start 1");
  assert(!sent[2] && !sent[6] && strstr(output,"START BLOCKED: arm ref set"));
  models[6].pos=-20000; cmd("arm ref set"); settle();
  assert(!arm_n && !triggers[2] && strstr(output,"ARM REF M6 zero=-20000 counts"));
  models[7].pos=0; cmd("arm ref check"); settle(); cmd("status");
  assert(!sent[2] && !arm_n && strstr(output,"@prep 1 2"));
  cmd("route start 1"); wait_text("ROUTE1: check QR");
  cmd("!"); settle(); cmd("arm ref status");
  assert(strstr(output,"ARM REF unset"));
  puts("PASS 74 route aliases require read-only zero capture and a fresh M5/M6 encoder check without enforcing M7 transport extension; stop invalidates it");
  reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); raw_station();
  shorten_lift=1U; cmd("route next"); wait_text("ROUTE1 rough"); tick(30000); settle();
  assert(arm_n==11U && triggers[2]==6U && ArmBench_HasFault() && MotorBench_HasFault() &&
         strstr(output,"M6 return to recorded top not verified"));
  puts("PASS 75 partial upward motion with the driver arrival flag never advances to the D return turn");
  reset(); setup_servos(CLAW_CLOSE,1500U); cmd("arm on"); raw_station();
  drift_before_turn=1U; cmd("route next"); wait_text("ROUTE1 rough"); settle();
  assert(!drift_before_turn && arm_n==11U && triggers[2]==6U &&
         strstr(output,"M5 rotation or M7 compensation blocked: M6 not at recorded top") && strstr(output,"@lift 9125 -1 0"));
  puts("PASS 76 a new M6 read blocks the D return turn even after the prior lift reached the recorded top");
  reset(); cmd("obs status"); assert(!RouteScene_Get()->valid);
  cmd("obs set 2 2 150 1200 30 350 250 25");
  assert(RouteScene_Get()->count==2U && RouteScene_Get()->obstacles[1].y_mm==250U);
  cmd("obs set 2 0"); cmd("obs set 1 0"); cmd("obs set 3 2 300 300 30 2600 100 20");
  cmd("obs set 3 0 extra"); cmd("obs set 3 5");
  assert(RouteScene_Get()->sequence==2U && RouteScene_Get()->count==2U);
  cmd("obs set 3 0"); assert(RouteScene_IsClear() && RouteScene_Get()->sequence==3U);
  puts("PASS 77 scene snapshots are atomic; stale, oversized and malformed input cannot clear the last valid scene");
  reset(); setup_servos(500U,1500U); cmd("arm on"); prepare_reference();
  cmd("route start 2");
  assert(!triggers[2] && !arm_n && !Mission_IsRunning());
  cmd("obs none"); cmd("route set arm_temp 60"); cmd("route start 2");
  assert(!triggers[2] && !Mission_IsRunning());
  cmd("route set arm_temp 179.5"); assert(RouteLibrary_StackDepth()==119500U);
  cmd("route reset"); assert(RouteLibrary_StackDepth()==98000U);
  cmd("obs set 1 1 1200 1200 50"); cmd("route start 2");
  assert(!triggers[2] && !Mission_IsRunning());
  cmd("obs none"); cmd("mission home 2"); cmd("route start 2");
  assert(!triggers[2] && !Mission_IsRunning());
  puts("PASS 78 route2 blocks before motion with unknown scene, obstacles, unsupported home or an invalid derived stack depth");
  {
    RouteStep step;
    assert(RouteLibrary_LegCount(ROUTE_RAW,ROUTE_ROUGH)==4U);
    assert(RouteLibrary_LegStep(ROUTE_RAW,ROUTE_ROUGH,0U,&step) && step.action==R_FORWARD && step.value==65U);
    assert(RouteLibrary_LegStep(ROUTE_RAW,ROUTE_ROUGH,1U,&step) && step.action==R_FORWARD && step.value==1700U);
    assert(RouteLibrary_LegStep(ROUTE_RAW,ROUTE_ROUGH,2U,&step) && step.action==R_TURN_RIGHT && step.value==180U);
    assert(RouteLibrary_LegStep(ROUTE_RAW,ROUTE_ROUGH,3U,&step) && step.action==R_BACKWARD && step.value==65U);
    assert(RouteLibrary_LegStep(ROUTE_ROUGH,ROUTE_TEMP,0U,&step) && step.action==R_FORWARD && step.value==10U);
    assert(RouteLibrary_LegStep(ROUTE_ROUGH,ROUTE_TEMP,1U,&step) && step.action==R_LEFT && step.value==850U);
    assert(RouteLibrary_LegStep(ROUTE_ROUGH,ROUTE_TEMP,2U,&step) && step.action==R_FORWARD && step.value==870U);
  }
  reset(); prepare_route2();
  models[5].pos=5000;
  before=sent[6];
  cmd("route start 2"); wait_text("STEP 2/2: forward 1050");
  assert(sent[6]==before);
  {
    const char frame[]="QR,156+123+516+231\n";
    feed(&pi,(const uint8_t *)frame,sizeof(frame)-1U);
  }
  wait_text("STOP: mission failed/timeout");
  assert(Mission_GetState()==MISSION_FAULT && !arm_n && stops[6]);
  puts("PASS route2 departs without an arm pose query and blocks unsafe M5/M6 posture at the first tray operation");
  reset(); prepare_route2(); models[7].pos=-42000; route_scan(2U); run_route2(0U,0U); tick(1);
  assert(!stops[2] && !stops[6]);
  puts("PASS 79 real route2 entry runs all 24 confirmed transfers through simulated motor/servo feedback with correct slots, ring order and stack depth");
  for (unsigned scenario=0;scenario<4;++scenario) route2_vision_trial(scenario);
  puts("PASS route2 vision bounded shift and rail restore, disabled budgets, invalid replies and held-object recheck");
  reset(); prepare_route2(); cmd("route start 2"); wait_text("CHASSIS RUN");
  before=triggers[2]; cmd("obs set 10 1 2100 600 25"); settle();
  assert(Mission_GetState()==MISSION_FAULT && stops[2] && stops[6] && triggers[2]==before);
  assert(RouteScene_Get()->sequence==10U && RouteScene_Get()->count==1U);
  cmd("obs set 11 0"); cmd("route start 2"); assert(triggers[2]==before);
  puts("PASS 80 obstacle snapshot during route2 stops both buses and later clearing never resumes movement");
  reset(); prepare_route2(); route_scan(2U); wait_text("@route2 1 ready load_raw");
  before=arm_n; cmd("route2 ack 1 fail"); settle();
  assert(arm_n==before && Mission_GetState()==MISSION_FAULT && stops[2] && stops[6]);
  cmd("mission status"); assert(strstr(output,"grabs=0 places=0"));
  puts("PASS 81 a rejected raw target stops before descent and gives no grab credit");
  reset(); prepare_route2(); cmd("route2 vision on"); cmd("route2 rawvision on"); cmd("route2 limits 0 0 10");
  route_scan(2U); run_route2(1U,0U);
  puts("PASS 82 two vision-enabled batches use top replies for pickup and both stack observations, restore M7, and move M6 98 mm each way");
  {
    const char *const trials[]={"raw_load","rough_place","rough_pick","temp_place","temp_stack"};
    const unsigned depths[]={3288U,6926U,6926U,6926U,4296U};
    for (unsigned trial=0U;trial<5U;++trial) {
      char command[64];
      MissionSnapshot task;
      reset(); prepare_route2();
      cmd("route2 vision on");
      snprintf(command,sizeof(command),"route2 trial %s 2",trials[trial]);
      cmd(command); wait_text("@route2 1 ready");
      assert(!arm_n && !triggers[2] && !pi_request[0]);
      cmd("route2 ack 1 ok"); wait_text("@route2 2 done");
      assert(arm_n==(trial==0U||trial==2U?4U:6U) && !triggers[2] && !pi_request[0] && Servo_OutputUs(1U)==1500U);
      assert(arm_pulses[trial==0U||trial==2U?1U:3U]==depths[trial]);
      cmd("route2 ack 2 ok"); wait_text("ROUTE2 ARM TRIAL DONE");
      Mission_GetSnapshot(&task);
      assert(task.state==MISSION_IDLE && !task.grabs && !task.places);
    }
    reset(); prepare_route2();
    cmd("route2 trial temp_stack 3"); wait_text("@route2 1 ready");
    cmd("route2 ack 1 fail"); settle();
    assert(!arm_n && !triggers[2] && Mission_GetState()==MISSION_FAULT);
  }
  puts("PASS 83 all five independent arm trials require ready/done and preserve the mission ledger");
  for (unsigned scenario=0;scenario<8U;++scenario) {
    unsigned id,tag;
    uint8_t shifts=(uint8_t)(scenario==0U || scenario==5U);
    unsigned passes=scenario==7U?5U:scenario==6U?3U:shifts||scenario==3U?2U:1U;
    reset(); prepare_route2();
    if (scenario==3U) models[7].pos=-63000; /* Real return pose outside the former 65 mm tolerance. */
    const int64_t pickup_rail_start=models[7].pos;
    cmd(scenario==1U||scenario==3U?"route2 pickup 2 0":scenario==5U?"route2 pickup 2 26":
      scenario==7U?"route2 pickup 2 30":"route2 pickup 2 20");
    wait_text("@route2 1 ready vision_pick");
    assert(!arm_n && !triggers[2] && !pi_request[0]);
    cmd("route2 ack 1 ok");
    for (unsigned pass=0;pass<passes;++pass) {
      for (unsigned t=0;t<15000U && !pi_request[0];++t) tick(1);
      assert(sscanf(pi_request,"VQ,%u,%u",&id,&tag)==2 && tag==1U && strstr(pi_request,",TOP"));
      pi_request[0]=0;
      assert(triggers[2]==(scenario==7U?pass:(shifts && pass>0U)));
      for (unsigned i=1;i<arm_n;++i) assert(arm_ids[i]==7U); /* M5 stays facing the part. */
      int forward=pass+1U<passes?8000:scenario==4U?25000:0;
      int left=pass==0U?(scenario==1U || scenario==5U?-24333:scenario<3U?-6000:0):0;
      if (scenario==3U) left=-5000;
      if (scenario==6U) {
        forward=pass==0U?15000:pass==1U?-3000:0;
        left=pass==0U?-3000:pass==1U?2333:0;
      }
      if (scenario==7U) { forward=0; left=-6000; }
      if (scenario==0U && pass==0U) {
        tick(4800);
        vision_reply(id,tag,0,0,3001U,0,1U); tick(20);
        assert(!triggers[2] && arm_n==1U);
        vision_reply(id,tag,forward,left,2300U,0,1U);
      } else vision_reply(id,tag,forward,left,0,0,scenario==2U?0U:1U);
    }
    if (shifts || scenario==3U || scenario==6U) {
      wait_text("@route2 2 done vision_pick");
      const unsigned pick=scenario==6U?3U:2U;
      assert(arm_n==pick+4U && triggers[2]==shifts);
      assert(arm_ids[1]==7U && arm_dirs[1]==0U && arm_pulses[1]==(scenario==6U?764U:407U));
      if (scenario==6U) {
        assert(arm_ids[2]==7U && arm_dirs[2]==1U && arm_pulses[2]==153U);
      }
      assert(arm_ids[pick]==6U && arm_pulses[pick]==6926U);
      assert(arm_ids[pick+2U]==7U && arm_dirs[pick+2U]==1U && arm_pulses[pick+2U]==(scenario==6U?611U:407U));
      assert(arm_ids[pick+3U]==5U);
      assert(llabs(models[7].pos-pickup_rail_start)<=1);
      assert(strstr(output,"PICKUP TEST M7 return origin="));
      if (scenario==0U) assert(strstr(output,"right 6 mm"));
      if (scenario==5U) assert(strstr(output,"right 24 mm"));
      cmd("route2 ack 2 ok"); wait_text("ROUTE2 ARM TRIAL DONE");
      assert(Mission_GetState()==MISSION_IDLE);
    } else {
      wait_text(scenario==1U?"travel budget exceeded":scenario==2U?"PICKUP TEST vision timeout":
        scenario==7U?"alignment did not converge":"forward chassis budget insufficient");
      if (scenario==1U) assert(strstr(output,"need 24 mm, remaining 0 mm (limit 0 mm)"));
      if (scenario==7U) {
        wait_text("STOP: arm trial failed;");
        assert(!strstr(output,"failed/timeout") && !strstr(output,"arm trial timeout"));
      }
      assert(triggers[2]==(scenario==7U?4U:0U) && arm_n==1U);
    }
  }
  puts("PASS 84 pickup trial uses asymmetric tolerances, rechecks TOP and restores the rail");
  for (unsigned scenario=0U;scenario<5U;++scenario) {
    unsigned requests=0U,id,tag;
    char command[80];
    reset(); prepare_route2();
    /* Singles use their own local commissioning budget, without a route scene. */
    cmd("obs set 1 1 1200 1200 50");
    models[7].pos=-42000;
    int64_t rail_start=models[7].pos;
    snprintf(command,sizeof(command),"route2 single %s 2 %s20",
      scenario==0U?"ground":scenario==2U?"stack":"ring",scenario==0U?"":"3 ");
    cmd(command);
    for (unsigned t=0U;t<100000U && !strstr(output,"ROUTE2 ARM TRIAL DONE") && !stops[6];++t) {
      if (pi_request[0]) {
        assert(sscanf(pi_request,"VQ,%u,%u",&id,&tag)==2);
        assert(tag==(scenario==0U?1U:3U));
        assert((strstr(pi_request,",TOP")!=NULL)==(scenario==0U || scenario==2U));
        pi_request[0]=0; ++requests;
        int forward=requests==1U?8000:0;
        int left=requests==1U?-6000:0;
        if (scenario==3U && requests==3U) left=6000; /* Held target shifted. */
        if (scenario==4U) {
          tick(11990U);
          assert(!stops[6] && !strstr(output,"vision timeout"));
          continue; /* Missing vision reply eventually stops the trial. */
        }
        if (scenario==1U && requests==1U) {
          tick(8000U);
          assert(!stops[6]);
        }
        vision_reply(id,tag,forward,left,0U,0U,(uint8_t)(scenario==0U || scenario==2U));
      }
      tick(1);
    }
    if (scenario<3U) {
      assert(strstr(output,"ROUTE2 ARM TRIAL DONE") && !stops[6] && !stops[2]);
      assert(requests==(scenario==0U?2U:3U) && triggers[2]==1U);
      assert(!strstr(output,"@route2 1 ready") && !strstr(output,"@route2 1 done"));
      assert(llabs(models[7].pos-rail_start)<=1 && !models[5].pos && !models[6].pos);
      assert(Mission_GetState()==MISSION_IDLE);
      unsigned ground_lifts=0,stack_lifts=0;
      for(unsigned j=0;j<arm_n;++j) if(arm_ids[j]==6U) {
        ground_lifts+=arm_pulses[j]==6926U;
        stack_lifts+=arm_pulses[j]==4296U;
      }
      assert(scenario==2U?stack_lifts==2U:ground_lifts==2U);
    } else {
      assert(stops[2] && stops[6] && !strstr(output,"ROUTE2 ARM TRIAL DONE"));
      if (scenario==3U) {
        assert(strstr(output,"@r2choice held"));
        for(unsigned j=0;j<arm_n;++j) assert(arm_ids[j]!=6U || arm_pulses[j]!=6926U);
      } else assert(strstr(output,"vision timeout"));
    }
  }
  puts("PASS 85 three visual singles auto-complete without route preconditions; ring/TOP replies, held-object gate, relative rail restore and vision timeout verified");
  for (unsigned scenario=0;scenario<4U;++scenario) {
    unsigned id,color;
    reset(); prepare_route2();
    cmd("route2 single raw 1 7");
    assert(!arm_n && !pi_request[0]);
    if (scenario==3U) cmd("route2 rawlimit 10");
    cmd("route2 single raw 2 6");
    wait_text("@r2decision load_raw wait_raw");
    assert(sscanf(pi_request,"VQ,%u,%u",&id,&color)==2 && color==6U && strstr(pi_request,",RAW"));
    assert(arm_n==1U && arm_ids[0]==5U && !triggers[2] && !moves[7]);
    vision_reply(id,color,0,0,0U,0U,1U); /* TOP reply cannot release a RAW wait. */
    vision_reply(id,5U,0,0,0U,0U,2U); /* Nor can a RAW reply for another color. */
    tick(12000U);
    assert(arm_n==1U && !stops[2] && !stops[6]);
    vision_reply(id,color,scenario?8000:5000,scenario>=2U?-6000:-5000,0U,0U,2U);
    if (scenario==2U) {
      wait_text("ROUTE2 raw chassis correction exceeds budget");
      assert(stops[2] && stops[6] && arm_n==1U);
    } else if (scenario) {
      unsigned refreshed;
      for (unsigned i=0U;i<20000U && !strstr(pi_request,scenario==3U?",RAW,8,-6":",RAW,8,0");++i)
        tick(1);
      assert(strstr(pi_request,scenario==3U?",RAW,8,-6":",RAW,8,0"));
      assert(sscanf(pi_request,"VQ,%u,%u",&refreshed,&color)==2 && refreshed!=id && color==6U);
      assert(arm_n==2U && arm_ids[1]==7U && moves[7]==1U);
      vision_reply(id,color,0,0,0U,0U,2U);
      tick(20U);
      assert(arm_n==2U);
      vision_reply(refreshed,color,0,0,0U,0U,2U);
      wait_text("ROUTE2 ARM TRIAL DONE");
      assert(moves[7]==2U && arm_n==6U && arm_ids[2]==6U && arm_ids[4]==7U);
      assert(triggers[2]==(scenario==3U?2U:0U));
    } else {
      wait_text("ROUTE2 ARM TRIAL DONE");
      assert(arm_n==4U && arm_ids[1]==6U && arm_pulses[1]==3288U);
      assert(strstr(output,"ROUTE2 raw 6 fwd=5000 left=-5000 um"));
      assert(!triggers[2] && !moves[7] && !models[5].pos && !models[6].pos);
      assert(!strstr(output,"@route2 1 ready") && Mission_GetState()==MISSION_IDLE);
    }
  }
  puts("PASS 86 raw single checks color and ID, corrects and restores M7 and chassis, and honors lateral budget");
  for (unsigned raw=0U;raw<2U;++raw) {
    unsigned id=0U,tag,old_id=0U;
    const unsigned target=raw?6U:1U;
    const uint8_t type=raw?2U:1U;
    int64_t rail_start;
    reset(); prepare_route2(); rail_start=models[7].pos;
    cmd(raw?"route2 single raw 2 6 100 20":"route2 single ground 2 100 20");
    for (unsigned round=0U;round<4U;++round) {
      for (unsigned i=0U;i<20000U && !pi_request[0];++i) tick(1);
      assert(sscanf(pi_request,"VQ,%u,%u",&id,&tag)==2 && id!=old_id && tag==target);
      assert(arm_n==(round==3U?2U:1U)); /* No descent before final reply. */
      assert(moves[7]==(round==3U?1U:0U));
      assert(triggers[2]==(round>1U?2U:round));
      if (raw) {
        const char *expected[]={",RAW\n",",RAW,17,0",",RAW,17,45",",RAW,27,45"};
        assert(strstr(pi_request,expected[round]));
      } else assert(strstr(pi_request,",TOP"));
      pi_request[0]=0;
      int forward=round==0U?27667:round<3U?10000:raw?-4877:0;
      int left=round<2U?45000:round==3U && raw?2775:0;
      vision_reply(id,tag,forward,left,0U,0U,type);
      old_id=id;
    }
    wait_text("ROUTE2 ARM TRIAL DONE");
    assert(triggers[2]==(raw?4U:2U) && moves[7]==2U && llabs(models[7].pos-rail_start)<=1);
    assert(strstr(output,"@r2choice foreaft 17") && strstr(output,"@r2choice shift -45"));
  }
  reset(); prepare_route2();
  cmd("route2 single ground 1 20 0");
  {
    unsigned id,tag;
    wait_text("@r2decision vision_pick pickup_result");
    assert(sscanf(pi_request,"VQ,%u,%u",&id,&tag)==2);
    vision_reply(id,tag,27000,6000,0U,0U,1U);
    wait_text("PICKUP TEST forward chassis budget insufficient");
    assert(!triggers[2] && !moves[7] && arm_n==1U);
  }
  puts("PASS joint ground/RAW: two chassis axes, fresh-frame M7, final descent, and zero-budget rejection");
  for (unsigned scenario=0U;scenario<3U;++scenario) {
    unsigned requests=0U,id,tag,last_id=0U;
    int sign=scenario==1U?-1:1;
    uint8_t top=(uint8_t)(scenario==1U);
    reset(); prepare_route2();
    int64_t rail_start=models[7].pos;
    cmd(top?"route2 single stack 2 3 20 20":"route2 single ring 2 3 20 20");
    for (unsigned t=0U;t<60000U && !strstr(output,"ROUTE2 ARM TRIAL DONE") && !stops[6];++t) {
      if (pi_request[0]) {
        assert(sscanf(pi_request,"VQ,%u,%u",&id,&tag)==2 && tag==3U && id!=last_id);
        assert((strstr(pi_request,",TOP")!=NULL)==top);
        last_id=id;
        pi_request[0]=0;
        ++requests;
        assert(triggers[2]==(requests>1U?1U:0U));
        if (requests<=3U) {
          for(unsigned j=0U;j<arm_n;++j) assert(arm_ids[j]!=6U);
          assert(moves[7]==(requests==3U?1U:0U));
        }
        vision_reply(id,tag,requests==1U?sign*27000:requests==2U?sign*10000:
                     requests==4U && scenario==2U?27000:0,0,0U,0U,top);
      }
      tick(1);
    }
    assert(requests==4U && triggers[2]==1U);
    assert(strstr(output,sign>0?"@r2choice foreaft 17":"@r2choice foreaft -17"));
    if (scenario==2U) {
      assert(stops[6] && strstr(output,"@r2choice held") && !strstr(output,"ROUTE2 ARM TRIAL DONE"));
      for(unsigned j=0U;j<arm_n;++j) assert(arm_ids[j]!=6U || arm_pulses[j]!=6926U);
    } else {
      assert(strstr(output,"ROUTE2 ARM TRIAL DONE") && moves[7]==4U);
      assert(llabs(models[7].pos-rail_start)<=1);
    }
  }
  reset(); prepare_route2();
  cmd("route2 pickup 1 20 20");
  wait_text("@route2 1 ready vision_pick");
  cmd("route2 ack 1 ok");
  wait_text("@r2decision vision_pick pickup_result");
  {
    unsigned id,tag;
    assert(sscanf(pi_request,"VQ,%u,%u",&id,&tag)==2 && tag==1U);
    vision_reply(id,tag,27000,0,0U,0U,1U);
    wait_text("@r2choice foreaft 17");
  }
  puts("PASS placement coarse correction: ring/stack and legacy pickup accept optional front travel; held recheck blocks descent");
  reset(); prepare_route2();
  models[6].pos=512;
  assert(ArmBench_RouteAction(1U,1U,0U,0U));
  settle();
  assert(!arm_n && stops[2] && stops[6] &&
         strstr(output,"Tray release height check failed: M6 not at recorded top"));
  puts("PASS 87 tray release at the working top sends no movement and faults if M6 is out of position");
  puts("ALL 87 INTEGRATION GROUPS PASSED; no serial device opened");
  return 0;
}
