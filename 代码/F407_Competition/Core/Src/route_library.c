#include "route_library.h"
#include "route_config.h"
#include "chassis_config.h"
#include "arm_config.h"
#include "arm_units.h"
#include "app_console.h"
#include <string.h>
#define PARAMS(X) \
 X(CENTER,center,0,10,3000) X(SCAN_GO,scan_go,0,10,3000) \
 X(SCAN_BACK,scan_back,0,10,3000) X(RAW_SHIFT,raw_shift,0,10,3000) \
 X(ROUGH_GO,rough_go,0,10,3000) X(ROUGH_TURN,rough_turn,1,1,360) \
 X(ROUGH_EXIT,rough_exit,0,10,3000) X(TEMP_SHIFT,temp_shift,0,10,3000) \
 X(TEMP_GO,temp_go,0,10,3000) X(TEMP_OFFSET,temp_offset,0,10,3000) \
 X(TEMP_TURN,temp_turn,1,1,360) X(HOME_TURN,home_turn,1,1,360) \
 X(HOME_OFFSET,home_offset,0,10,3000) X(HOME_GO,home_go,0,10,3000) \
 X(HOME_CROSS,home_cross,0,10,3000) X(HOME_ENTRY,home_entry,0,10,3000) \
 X(HOME_SHIFT,home_shift,0,10,3000) \
 X(DRIVE_RPM,drive_rpm,2,0,CHASSIS_MAX_RPM) X(TURN_RPM,turn_rpm,2,0,CHASSIS_MAX_RPM) \
 X(ARM_TURN,arm_turn,3,1,ARM_M5_MAX_TRAVEL_MILLI) \
 X(ARM_RAW,arm_raw,4,1,ARM_M6_MAX_TRAVEL_MILLI) \
 X(ARM_PICK,arm_pick,4,1,ARM_M6_MAX_TRAVEL_MILLI) \
 X(ARM_ROUGH_PLACE,arm_rough_place,4,1,ARM_M6_MAX_TRAVEL_MILLI) \
 X(ARM_TEMP,arm_temp,4,1,ARM_M6_MAX_TRAVEL_MILLI) \
 X(ARM_DROP,arm_drop,4,0,ARM_M6_MAX_TRAVEL_MILLI) \
 X(ARM_TRAY_PICK,arm_tray_pick,4,1,ARM_M6_MAX_TRAVEL_MILLI) \
 X(ARM_RPM,arm_rpm,2,0,ARM_M5_MAX_RPM)
#define ENUM(k,n,u,lo,hi) P_##k,
enum { PARAMS(ENUM) P_COUNT };
#undef ENUM
#define META(k,n,u,lo,hi) {#n,"RP_" #k,RP_##k,lo,hi,u},
static const struct { const char *name,*macro; uint32_t initial,min,max; uint8_t unit; } params[]={PARAMS(META)};
#undef META
static uint32_t values[P_COUNT];
_Static_assert(P_COUNT<255,"Too many named route parameters");
/* EDITABLE LIBRARIES. One level of route -> segment references; no recursive
 * calls, heap or script parser. All actions use named parameters above. */
typedef struct { uint8_t action,param,speed,group,flags; } Entry;
#define MOVE(a,p,s) {R_##a,P_##p,P_##s,0,0}
#define EVENT(a,g) {R_##a,R_NO_PARAM,R_NO_PARAM,g,0}
enum { RAW_LOAD=1,ROUGH_TRANSFER,TEMP_UNLOAD };
static const Entry to_scan[]={MOVE(RIGHT,CENTER,DRIVE_RPM),
  {R_FORWARD,P_SCAN_GO,P_DRIVE_RPM,0,R_CLEAR_QR},EVENT(SCAN,0)};
static const Entry to_raw[]={MOVE(BACKWARD,SCAN_BACK,DRIVE_RPM),MOVE(RIGHT,RAW_SHIFT,DRIVE_RPM),EVENT(RAW,RAW_LOAD)};
static const Entry to_rough[]={MOVE(FORWARD,ROUGH_GO,DRIVE_RPM),MOVE(TURN_RIGHT,ROUGH_TURN,TURN_RPM),EVENT(WORK,ROUGH_TRANSFER)};
static const Entry to_temp[]={MOVE(BACKWARD,ROUGH_EXIT,DRIVE_RPM),MOVE(LEFT,TEMP_SHIFT,DRIVE_RPM),
  MOVE(FORWARD,TEMP_GO,DRIVE_RPM),MOVE(RIGHT,TEMP_OFFSET,DRIVE_RPM),MOVE(TURN_RIGHT,TEMP_TURN,TURN_RPM),EVENT(TEMP,TEMP_UNLOAD)};
static const Entry to_home[]={MOVE(TURN_LEFT,HOME_TURN,TURN_RPM),MOVE(LEFT,HOME_OFFSET,DRIVE_RPM),
  MOVE(FORWARD,HOME_GO,DRIVE_RPM),MOVE(RIGHT,HOME_CROSS,DRIVE_RPM),
  MOVE(FORWARD,HOME_ENTRY,DRIVE_RPM),MOVE(RIGHT,HOME_SHIFT,DRIVE_RPM)};
#define ARRAY(a) a,(uint8_t)(sizeof(a)/sizeof((a)[0]))
#ifdef ROUTE_LIBRARY_TEST
static const Entry bad_nav[]={{R_FORWARD,P_ARM_TURN,P_DRIVE_RPM,0,0}},bad_work[]={EVENT(WORK,99)};
#endif
/* Pose labels are contracts, not measured localization. */
enum { START_DOWN,SCAN_DOWN,RAW_DOWN,ROUGH_UP,TEMP_RIGHT,HOME_UP };
static const char *const poses[]={"start/down","scan/down","raw/down","rough/up","temp/right","home/up"};
static const struct { const Entry *steps; uint8_t count; const char *name; uint8_t from,to; } segments[]={
  {ARRAY(to_scan),"to_scan",START_DOWN,SCAN_DOWN},{ARRAY(to_raw),"to_raw",SCAN_DOWN,RAW_DOWN},{ARRAY(to_rough),"to_rough",RAW_DOWN,ROUGH_UP},
  {ARRAY(to_temp),"to_temp",ROUGH_UP,TEMP_RIGHT},{ARRAY(to_home),"to_home",TEMP_RIGHT,HOME_UP}
#ifdef ROUTE_LIBRARY_TEST
  ,{ARRAY(bad_nav),"bad_nav",START_DOWN,HOME_UP},{ARRAY(bad_work),"bad_work",START_DOWN,HOME_UP}
#endif
};
/* Add a path by adding its segment list and directory row. No executor change.
 * Segment contracts are measured start/end poses; sharing does NOT prove fit. */
#ifdef ROUTE_LIBRARY_TEST
/* Synthetic host fixtures, never registered in production firmware. */
static const uint8_t path2[]={0},bad_path[]={255},bad_nav_path[]={5},bad_work_path[]={6},bad_join_path[]={2,0};
#endif
static const uint8_t path1[]={0,1,2,3,4};
static const struct { const uint8_t *segments; uint8_t count,id; const char *name; } paths[]={
  {ARRAY(path1),1,"route1"}
#ifdef ROUTE_LIBRARY_TEST
  ,{ARRAY(path2),7,"fixture_scan_only"},{ARRAY(bad_path),3,"fixture_bad_segment"},
  {ARRAY(bad_nav_path),4,"fixture_bad_nav"},{ARRAY(bad_work_path),5,"fixture_bad_work"},{ARRAY(bad_join_path),6,"fixture_bad_join"}
#endif
};
typedef struct { uint8_t axis,negative,claw,param,speed; TransferStep step; } WorkEntry;
#define TURN(ccw) {0,ccw,0,P_ARM_TURN,P_ARM_RPM,(ccw)?FACE_TRAY:FACE_STATION}
#define LIFT(down,p,step) {1,down,0,P_##p,P_ARM_RPM,step}
#define CLAW(state,step) {255,0,state,R_NO_PARAM,R_NO_PARAM,step}
#define GRAB(p) LIFT(1,p,LOWER_PICK),CLAW(2,GRIP),LIFT(0,p,LIFT_PICK)
#define PLACE(p) LIFT(1,p,LOWER_PLACE),CLAW(1,RELEASE),LIFT(0,p,LIFT_PLACE)
/* Begin and end each group at transport height, facing the fixed tray slot.
 * GRAB starts with the claw open; PLACE releases before lifting. */
static const WorkEntry raw_load[]={
  TURN(0),CLAW(1,OPEN_CLAW),GRAB(ARM_RAW),TURN(1),PLACE(ARM_DROP)};
static const WorkEntry rough_transfer[]={
  GRAB(ARM_TRAY_PICK),TURN(0),PLACE(ARM_ROUGH_PLACE),GRAB(ARM_PICK),TURN(1),PLACE(ARM_DROP)};
static const WorkEntry temp_unload[]={
  GRAB(ARM_TRAY_PICK),TURN(0),PLACE(ARM_TEMP),TURN(1)};
static const struct { const WorkEntry *steps; uint8_t count,id; const char *name; } groups[]={
  {ARRAY(raw_load),RAW_LOAD,"raw_load"},
  {ARRAY(rough_transfer),ROUGH_TRANSFER,"rough_transfer"},
  {ARRAY(temp_unload),TEMP_UNLOAD,"temp_unload"}};
#define N(a) (sizeof(a)/sizeof((a)[0]))
static const char *const units[]={"mm","deg","rpm(0=inherit)","deg","mm"};
static const char *const actions[]={"forward","backward","left","right","turn-left","turn-right","scan","raw-wait","rough","temp"};
static uint8_t PathIndex(uint8_t id) { unsigned i; for(i=0;i<N(paths);++i) if(paths[i].id==id) return (uint8_t)i; return 255; }
static uint8_t GroupIndex(uint8_t id) { unsigned i; for(i=0;i<N(groups);++i) if(groups[i].id==id) return (uint8_t)i; return 255; }
void RouteLibrary_Init(void) { unsigned i; for(i=0;i<P_COUNT;++i) values[i]=params[i].initial; }
uint8_t RouteLibrary_Count(uint8_t id) {
  unsigned total=0,i; uint8_t p=PathIndex(id); if(p==255) return 0;
  for(i=0;i<paths[p].count;++i) { uint8_t s=paths[p].segments[i]; if(s>=N(segments)) return 0; total+=segments[s].count; }
  return total && total<=255U?(uint8_t)total:0;
}
uint8_t RouteLibrary_Step(uint8_t id,uint8_t index,RouteStep *out) {
  unsigned i; uint8_t p=PathIndex(id); if(p==255 || !out) return 0;
  for(i=0;i<paths[p].count;++i) {
    uint8_t s=paths[p].segments[i]; const Entry *e; if(s>=N(segments)) return 0;
    if(index>=segments[s].count) { index-=segments[s].count; continue; }
    e=&segments[s].steps[index];
    if(e->action>R_TEMP || (e->param!=255 && e->param>=P_COUNT) || (e->speed!=255 && e->speed>=P_COUNT)) return 0;
    if(e->action<=R_TURN_RIGHT && (e->param==255 || params[e->param].unit!=(e->action<R_TURN_LEFT?0:1))) return 0;
    if(e->speed!=255 && params[e->speed].unit!=2) return 0;
    out->value=e->param==255?0:(uint16_t)values[e->param]; out->rpm=e->speed==255?0:(uint16_t)values[e->speed];
    out->action=e->action; out->group=e->group; out->flags=e->flags; return 1;
  } return 0;
}
uint8_t RouteLibrary_WorkCount(uint8_t id) { uint8_t g=GroupIndex(id); return g==255?0:groups[g].count; }
uint8_t RouteLibrary_Work(uint8_t id,uint8_t index,RouteWorkStep *out) {
  uint8_t g=GroupIndex(id); const WorkEntry *e;
  if(!out || g==255 || index>=groups[g].count) return 0;
  e=&groups[g].steps[index];
  if((e->param!=255 && e->param>=P_COUNT) || (e->speed!=255 && e->speed>=P_COUNT)) return 0;
  if(e->axis!=255 && (e->axis>2 || e->param==255 || params[e->param].unit!=(e->axis==0?3:4))) return 0;
  if(e->speed!=255 && params[e->speed].unit!=2) return 0;
  out->amount=e->param==255?0:values[e->param]; out->rpm=e->speed==255?0:(uint16_t)values[e->speed];
  out->axis=e->axis; out->negative=e->negative; out->claw=e->claw; return 1;
}
/* Route 2 has its own fixed work/turn poses; route1 geometry stays unchanged.
 * Movement tables contain no station actions: the scheduler owns those. */
typedef struct { uint8_t action,flags; uint16_t value; } LegEntry;
#define NAV(a,v) {R_##a,0U,v}
static const LegEntry route2_to_scan[]={NAV(RIGHT,RP_CENTER),{R_FORWARD,R_CLEAR_QR,RP_SCAN_GO}};
static const LegEntry route2_to_raw[]={NAV(BACKWARD,RP_SCAN_BACK),NAV(RIGHT,RP_RAW_SHIFT),
  NAV(BACKWARD,ROUTE1_TOP_Y-ROUTE2_RAW_Y)};
static const LegEntry route2_to_rough[]={NAV(FORWARD,ROUTE1_TOP_Y-ROUTE2_RAW_Y),
  NAV(FORWARD,ROUTE2_ROUGH_TURN_Y-ROUTE1_TOP_Y),NAV(TURN_RIGHT,180U),
  NAV(BACKWARD,ROUTE2_ROUGH_Y-ROUTE2_ROUGH_TURN_Y)};
static const LegEntry route2_to_temp[]={NAV(FORWARD,ROUTE2_ROUGH_Y-ROUTE2_LOWER_Y),
  NAV(LEFT,RP_TEMP_SHIFT),NAV(FORWARD,ROUTE2_LOWER_Y-ROUTE1_SCAN_Y),
  NAV(RIGHT,ROUTE2_TEMP_TURN_X-ROUTE1_LEFT_X),NAV(TURN_RIGHT,90U),
  NAV(BACKWARD,ROUTE2_TEMP_TURN_X-ROUTE2_TEMP_X)};
static const LegEntry route2_back_to_raw[]={NAV(FORWARD,ROUTE2_TEMP_TURN_X-ROUTE2_TEMP_X),
  NAV(TURN_RIGHT,90U),NAV(RIGHT,ROUTE2_TEMP_TURN_X-ROUTE1_LEFT_X),
  NAV(BACKWARD,RP_SCAN_BACK),NAV(LEFT,RP_TEMP_SHIFT),NAV(BACKWARD,ROUTE1_TOP_Y-ROUTE2_RAW_Y)};
static const LegEntry route2_to_home[]={NAV(FORWARD,ROUTE2_TEMP_TURN_X-ROUTE2_TEMP_X),
  NAV(TURN_LEFT,90U),NAV(LEFT,ROUTE2_TEMP_TURN_X-ROUTE1_LEFT_X),
  NAV(FORWARD,RP_HOME_GO),NAV(RIGHT,RP_HOME_CROSS),NAV(FORWARD,RP_HOME_ENTRY),NAV(RIGHT,RP_HOME_SHIFT)};
#undef NAV
/* Station events belong to the mission scheduler. A leg is keyed by both endpoint poses. */
static const LegEntry *Leg(RouteStation from,RouteStation to,uint8_t *count) {
  static const struct { const LegEntry *steps; uint8_t count; RouteStation from,to; } legs[]={
    {ARRAY(route2_to_scan),ROUTE_START,ROUTE_SCAN},
    {ARRAY(route2_to_raw),ROUTE_SCAN,ROUTE_RAW},
    {ARRAY(route2_to_rough),ROUTE_RAW,ROUTE_ROUGH},
    {ARRAY(route2_to_temp),ROUTE_ROUGH,ROUTE_TEMP},
    {ARRAY(route2_back_to_raw),ROUTE_TEMP,ROUTE_RAW},
    {ARRAY(route2_to_home),ROUTE_TEMP,ROUTE_HOME}
  };
  unsigned i;
  for(i=0;i<N(legs);++i) if(legs[i].from==from && legs[i].to==to) {
    *count=legs[i].count;
    return legs[i].steps;
  }
  *count=0U;
  return NULL;
}
uint8_t RouteLibrary_LegCount(RouteStation from,RouteStation to) {
  uint8_t count;
  (void)Leg(from,to,&count);
  return count;
}
uint8_t RouteLibrary_LegStep(RouteStation from,RouteStation to,uint8_t index,RouteStep *out) {
  uint8_t count;
  const LegEntry *leg=Leg(from,to,&count);
  if(!out || index>=count) return 0U;
  const LegEntry *e=&leg[index];
  if(e->action>R_TURN_RIGHT || e->value<(e->action<R_TURN_LEFT?10U:1U) ||
      e->value>(e->action<R_TURN_LEFT?3000U:360U)) return 0U;
  *out=(RouteStep){e->value,(uint16_t)values[e->action<R_TURN_LEFT?P_DRIVE_RPM:P_TURN_RPM],e->action,0U,e->flags};
  return 1U;
}
uint8_t RouteLibrary_Route2Ready(void) {
  unsigned i;
  RouteStep step;
  /* Route2 uses the design's fixed station poses and coarse offsets. Changes
   * to route1-only geometry require revisiting the route2 envelope first. */
  for(i=P_CENTER;i<=P_HOME_SHIFT;++i) if(values[i]!=params[i].initial) return 0U;
  for(RouteStation from=ROUTE_START;from<=ROUTE_HOME;++from)
    for(RouteStation to=ROUTE_START;to<=ROUTE_HOME;++to)
      for(i=0;i<RouteLibrary_LegCount(from,to);++i)
        if(!RouteLibrary_LegStep(from,to,(uint8_t)i,&step)) return 0U;
  return (uint8_t)(values[P_ARM_TURN]==RP_ARM_TURN && RouteLibrary_Validate(1U));
}
uint32_t RouteLibrary_StackDepth(void) {
  const uint32_t part_height=60000U; /* 60 mm in thousandths of a millimetre. */
  return values[P_ARM_TEMP]>part_height?values[P_ARM_TEMP]-part_height:0U;
}
TransferStep RouteLibrary_Transfer(RouteStation station,uint8_t loading,uint32_t depth,
                              uint8_t index,RouteWorkStep *out) {
  const WorkEntry *e;
  if (loading) {
    if (station!=ROUTE_RAW && station!=ROUTE_ROUGH) return WORK_NONE;
    if (index>=N(raw_load)) return index==N(raw_load)?WORK_COMPLETE:WORK_NONE;
    e=&raw_load[index];
  } else {
    if (station!=ROUTE_ROUGH && station!=ROUTE_TEMP) return WORK_NONE;
    if (index>=N(temp_unload)) return index==N(temp_unload)?WORK_COMPLETE:WORK_NONE;
    e=&temp_unload[index];
  }
  if (!out) return e->step;
  uint32_t amount=e->param==R_NO_PARAM?0U:values[e->param];
  if(e->param==P_ARM_RAW && station==ROUTE_ROUGH) amount=values[P_ARM_PICK];
  if(e->param==P_ARM_TEMP) amount=depth?depth:values[station==ROUTE_ROUGH?P_ARM_ROUGH_PLACE:P_ARM_TEMP];
  *out=(RouteWorkStep){amount,e->speed==R_NO_PARAM?0U:(uint16_t)values[e->speed],
                      e->axis,e->negative,e->claw};
  return e->step;
}
uint8_t RouteLibrary_Validate(uint8_t id) {
  unsigned i,j; RouteStep s; RouteWorkStep w; uint8_t count=RouteLibrary_Count(id);
  if(!count) return 0;
  { uint8_t p=PathIndex(id),previous=255;
    for(i=0;i<paths[p].count;++i) {
      uint8_t k=paths[p].segments[i];
      if(k>=N(segments) || segments[k].from>=N(poses) || segments[k].to>=N(poses) ||
          (previous!=255 && previous!=segments[k].from)) return 0;
      previous=segments[k].to;
    }
  }
  for(i=0;i<P_COUNT;++i) if(values[i]<params[i].min || values[i]>params[i].max) return 0;
  for(i=0;i<count;++i) {
    if(!RouteLibrary_Step(id,(uint8_t)i,&s) || s.flags>R_CLEAR_QR) return 0;
    if(s.action<=R_TURN_RIGHT) {
      if(s.value<(s.action<R_TURN_LEFT?10U:1U) || s.value>(s.action<R_TURN_LEFT?3000U:360U) || s.rpm>CHASSIS_MAX_RPM || s.group) return 0;
    } else if(s.flags || s.rpm || s.value || (s.action==R_SCAN && s.group)) return 0;
    if(!s.group && s.action!=R_WORK) continue;
    if(!RouteLibrary_WorkCount(s.group)) return 0;
    for(j=0;j<RouteLibrary_WorkCount(s.group);++j) {
      static const uint32_t caps[]={ARM_M5_MAX_TRAVEL_MILLI,ARM_M6_MAX_TRAVEL_MILLI,ARM_M7_MAX_TRAVEL_MILLI};
      static const uint16_t speeds[]={ARM_M5_MAX_RPM,ARM_M6_MAX_RPM,ARM_M7_MAX_RPM};
      if(!RouteLibrary_Work(s.group,(uint8_t)j,&w) || w.claw>2 || w.negative>1) return 0;
      if(w.axis==255) { if(!w.claw || w.amount || w.rpm) return 0; }
      else if(w.axis>2 || (!w.amount && w.axis!=1U) || w.amount>caps[w.axis] || w.rpm>speeds[w.axis]) return 0;
    }
  } return 1;
}
static void PrintValue(unsigned i) {
  if(params[i].unit>=3) App_Log("%lu.%03lu",(unsigned long)(values[i]/1000U),(unsigned long)(values[i]%1000U));
  else App_Log("%lu",(unsigned long)values[i]);
}
static void Show(uint8_t id) {
  unsigned i,j; uint8_t p=PathIndex(id); RouteStep s;
  if(p==255 || !RouteLibrary_Validate(id)) { App_Print("Invalid path/config.\r\n"); return; }
  App_Log("route %u %s; %u steps; shared RAM parameters:\r\n",id,paths[p].name,RouteLibrary_Count(id));
  for(i=0;i<P_COUNT;++i) { App_Log("%s=",params[i].name); PrintValue(i); App_Log(" %s\r\n",units[params[i].unit]); }
  for(i=0;i<paths[p].count;++i) {
    uint8_t k=paths[p].segments[i]; App_Log("segment %u %s [%s -> %s]\r\n",k,segments[k].name,poses[segments[k].from],poses[segments[k].to]);
  }
  for(i=0;i<RouteLibrary_Count(id);++i) {
    (void)RouteLibrary_Step(id,(uint8_t)i,&s);
    App_Log("%u %s value=%u rpm=%u%s\r\n",i+1,actions[s.action],s.value,s.rpm,s.flags?" clear-QR":"");
    if(s.group) {
      uint8_t g=GroupIndex(s.group); App_Log(" group %u %s\r\n",s.group,groups[g].name);
      for(j=0;j<groups[g].count;++j) {
        RouteWorkStep w; (void)RouteLibrary_Work(s.group,(uint8_t)j,&w);
        static const char *const axis[]={"rotate","lift","rail"},*const pos[]={"cw","up","forward"},*const neg[]={"ccw","down","backward"};
        if(w.axis!=255) App_Log("  %u %s %s %lu.%03lu %s rpm=%u",j+1,axis[w.axis],w.negative?neg[w.axis]:pos[w.axis],
          (unsigned long)(w.amount/1000U),(unsigned long)(w.amount%1000U),w.axis?"mm":"deg",w.rpm);
        else App_Log("  %u",j+1);
        if(w.claw) App_Log(" + claw %s",w.claw==2?"close":"open");
        App_Print("\r\n");
      }
    }
  }
  App_Print("Preview only; RPM 0 inherits current module settings. Poses/clearance require calibration.\r\n");
}
void RouteLibrary_Command(const char *command) {
  char line[96],*op,*a,*b,*extra; uint32_t v; unsigned i;
  if(!command || strlen(command)>=sizeof(line)) goto invalid;
  memcpy(line,command,strlen(command)+1); op=strtok(line," "); a=strtok(NULL," "); b=strtok(NULL," "); extra=strtok(NULL," ");
  if(!op) {
    for(i=0;i<N(paths);++i) App_Log("route %u %s (%u steps)\r\n",paths[i].id,paths[i].name,RouteLibrary_Count(paths[i].id));
    return;
  }
  if(!strcmp(op,"show") && !b && (!a || App_ParseU32(a,1,255,&v))) { Show(a?(uint8_t)v:1U); return; }
  if(!strcmp(op,"export") && !a) {
    App_Print("/* Replace matching RP_ definitions in route_config.h; arm units are 0.001mm/deg. */\r\n");
    for(i=0;i<P_COUNT;++i) App_Log("#define %s %luU\r\n",params[i].macro,(unsigned long)values[i]);
    return;
  }
  if(!strcmp(op,"reset") && !a) { RouteLibrary_Init(); App_Print("Route defaults restored.\r\n"); return; }
  if(!strcmp(op,"set") && a && b && !extra) {
    for(i=0;i<P_COUNT;++i) if(!strcmp(a,params[i].name)) {
      if(params[i].unit>=3) { if(!ArmUnits_ParseMilli(b,&v)) goto invalid; }
      else if(!App_ParseU32(b,params[i].min,params[i].max,&v)) goto invalid;
      if(v<params[i].min || v>params[i].max) goto invalid;
      values[i]=v; App_Log("RAM %s=",a); PrintValue(i); App_Log(" %s\r\n",units[params[i].unit]); return;
    }
  }
invalid: App_Print("route [show [ID]|set NAME VALUE|reset|export|start ID|next|status|stop]\r\n");
}
