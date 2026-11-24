#include "route_scene.h"
#include "app_console.h"
#include <string.h>

static RouteScene scene;

void RouteScene_Init(void) { memset(&scene,0,sizeof(scene)); }
const RouteScene *RouteScene_Get(void) { return &scene; }
uint8_t RouteScene_IsClear(void) { return scene.valid && !scene.count; }
uint8_t RouteScene_AllowLeg(RouteStation from,RouteStation to) {
  return RouteScene_IsClear() && RouteLibrary_LegCount(from,to);
}

uint8_t RouteScene_Command(const char *command,uint8_t allow_none) {
  RouteScene next={0};
  char line[160],*token;
  uint32_t count,x,y,radius;
  unsigned i;
  if (!strcmp(command,"status")) {
    App_Log("@obs %lu %u %u\r\n",(unsigned long)scene.sequence,scene.valid,scene.count);
    for(i=0;i<scene.count;++i) App_Log("@obstacle %u %u %u\r\n",
      scene.obstacles[i].x_mm,scene.obstacles[i].y_mm,scene.obstacles[i].radius_mm);
    return 0U;
  }
  if (!strcmp(command,"none") && allow_none) {
    next.sequence=scene.sequence; next.valid=1U; scene=next;
    App_Print("OBS NONE\r\n"); return 0U;
  }
  if (strncmp(command,"set ",4U) || strlen(command+4U)>=sizeof(line)) goto invalid;
  strcpy(line,command+4U);
  token=strtok(line," ");
  if (!App_ParseU32(token,1U,UINT32_MAX,&next.sequence)) goto invalid;
  if (next.sequence<=scene.sequence) { App_Print("OBS REJECT sequence\r\n"); return 0U; }
  if (!App_ParseU32(strtok(NULL," "),0U,ROUTE_OBSTACLE_CAPACITY,&count)) goto invalid;
  for(i=0;i<count;++i) {
    if (!App_ParseU32(strtok(NULL," "),0U,2400U,&x) ||
        !App_ParseU32(strtok(NULL," "),0U,2400U,&y) ||
        !App_ParseU32(strtok(NULL," "),1U,2400U,&radius)) goto invalid;
    next.obstacles[i]=(RouteObstacle){(uint16_t)x,(uint16_t)y,(uint16_t)radius};
  }
  if (strtok(NULL," ")) goto invalid;
  next.count=(uint8_t)count; next.valid=1U; scene=next;
  App_Log("OBS OK %lu %u\r\n",(unsigned long)scene.sequence,scene.count);
  return scene.count!=0U;
invalid:
  App_Print("OBS REJECT use obs none/status/set SEQ COUNT [X Y R ...]\r\n");
  return 0U;
}
