/* Reuse the existing simulated hardware; no real UART or motor is opened. */
#define main previous_regression_main
#include "test_competition.c"
#undef main

static void verify_mission(unsigned raw_enabled,unsigned no_lift) {
  reset(); prepare_route2(); cmd("route2 vision on");
  cmd(raw_enabled?"route2 rawvision on":"route2 rawvision off");
  route_scan(no_lift?3U:2U);
  unsigned cursor=0,items=0,raw=0,top=0,ring=0;
  for(unsigned t=0;t<400000 && Mission_IsRunning();++t) {
    tick(1);
    if(pi_request[0]) {
      unsigned id,target;
      assert(sscanf(pi_request,"VQ,%u,%u",&id,&target)==2);
      unsigned kind=strstr(pi_request,",RAW")?2U:strstr(pi_request,",TOP")?1U:0U;
      if(kind==2U) { assert(raw_enabled); ++raw; }
      else if(kind) ++top; else ++ring;
      pi_request[0]=0;
      /* Exact 5mm boundaries must pass without corrective moves. */
      vision_reply(id,target,5000,-5000,0,0,(uint8_t)kind);
    }
    const char *event=strstr(output+cursor,"@route2 ");
    if(event) {
      unsigned id; char command[48];
      assert(sscanf(event,"@route2 %u",&id)==1);
      if(strstr(event," done ") && strstr(event," done ")<strchr(event,'\n')) ++items;
      cursor=(unsigned)(strchr(event,'\n')-output+1);
      snprintf(command,sizeof(command),"route2 ack %u ok",id); cmd(command);
    }
  }
  MissionSnapshot snapshot; Mission_GetSnapshot(&snapshot);
  if(snapshot.state!=MISSION_DONE) fprintf(stderr,"%s\n",output+(used>3000?used-3000:0));
  assert(snapshot.state==MISSION_DONE && items==24U);
  /* One observation per transfer, never another with the claw occupied. */
  assert(raw==(raw_enabled?6U:0U) && top==9U && ring==9U);
  assert(snapshot.grabs==12 && snapshot.places==12 && !moves[7]);
  if(no_lift) assert(!moves[6]);
}

static void verify_over_tolerance(void) {
  reset(); prepare_route2(); cmd("route2 pickup 2 20");
  wait_text("@route2 1 ready vision_pick"); cmd("route2 ack 1 ok");
  unsigned id,target;
  for(unsigned t=0;t<15000 && !pi_request[0];++t) tick(1);
  assert(sscanf(pi_request,"VQ,%u,%u",&id,&target)==2);
  pi_request[0]=0;
  vision_reply(id,target,6000,0,0,0,1U);
  for(unsigned t=0;t<15000 && !pi_request[0];++t) tick(1);
  assert(pi_request[0] && moves[7]);
  assert(sscanf(pi_request,"VQ,%u,%u",&id,&target)==2); pi_request[0]=0;
  vision_reply(id,target,5000,5000,0,0,1U);
  wait_text("@route2 2 done vision_pick");
  cmd("!"); settle();
}

int main(void) {
  verify_mission(1U,0U);
  puts("PASS route2: 5mm boundaries, 24 transfers, one observation each, no held recheck");
  verify_mission(0U,1U);
  puts("PASS route3: rawvision off sends no RAW, other vision retained, M6 motion skipped");
  verify_over_tolerance();
  puts("PASS 6mm needs correction and a new TOP request; 5mm then accepted");
  return 0;
}
