#ifndef ROUTE_SCENE_H
#define ROUTE_SCENE_H
#include "route_library.h"
#define ROUTE_OBSTACLE_CAPACITY 4U
typedef struct { uint16_t x_mm,y_mm,radius_mm; } RouteObstacle;
typedef struct {
  uint32_t sequence;
  uint8_t valid,count;
  RouteObstacle obstacles[ROUTE_OBSTACLE_CAPACITY];
} RouteScene;
void RouteScene_Init(void);
const RouteScene *RouteScene_Get(void);
uint8_t RouteScene_IsClear(void);
uint8_t RouteScene_AllowLeg(RouteStation from,RouteStation to);
/* Returns 1 only for an accepted nonempty snapshot. */
uint8_t RouteScene_Command(const char *command,uint8_t allow_none);
#endif
