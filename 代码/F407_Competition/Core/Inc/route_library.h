#ifndef ROUTE_LIBRARY_H
#define ROUTE_LIBRARY_H
#include <stdint.h>
/* Values 0..9 preserve existing chassis motion/station semantics. */
enum { R_FORWARD,R_BACKWARD,R_LEFT,R_RIGHT,R_TURN_LEFT,R_TURN_RIGHT,
       R_SCAN,R_RAW,R_WORK,R_TEMP };
enum { R_NO_PARAM=255, R_CLEAR_QR=1 };
typedef struct { uint16_t value,rpm; uint8_t action,group,flags; } RouteStep;
typedef enum { ROUTE_START, ROUTE_SCAN, ROUTE_RAW, ROUTE_ROUGH, ROUTE_TEMP, ROUTE_HOME } RouteStation;
/* axis 0/1/2=M5/M6/M7, 255=no motor; negative=ccw/down/backward;
 * claw 0=none,1=open,2=close; same row motor+claw starts concurrently. */
typedef struct { uint32_t amount; uint16_t rpm; uint8_t axis,negative,claw; } RouteWorkStep;
void RouteLibrary_Init(void);
uint8_t RouteLibrary_Count(uint8_t id);
uint8_t RouteLibrary_Step(uint8_t id,uint8_t index,RouteStep *out);
uint8_t RouteLibrary_Work(uint8_t group,uint8_t index,RouteWorkStep *out);
uint8_t RouteLibrary_WorkCount(uint8_t group);
uint8_t RouteLibrary_Validate(uint8_t id);
uint8_t RouteLibrary_LegCount(RouteStation from,RouteStation to);
uint8_t RouteLibrary_LegStep(RouteStation from,RouteStation to,uint8_t index,RouteStep *out);
uint8_t RouteLibrary_Route2Ready(void);
uint32_t RouteLibrary_StackDepth(void);
typedef enum {
  WORK_NONE, FACE_STATION, OPEN_CLAW, LOWER_PICK, GRIP, LIFT_PICK,
  FACE_TRAY, LOWER_PLACE, RELEASE, LIFT_PLACE, WORK_COMPLETE, RAIL_APPLY, RAIL_RESTORE
} TransferStep;
/* WORK_NONE is invalid. WORK_COMPLETE indicates the end of a valid sequence. */
TransferStep RouteLibrary_Transfer(RouteStation station,uint8_t loading,uint32_t depth,
                              uint8_t index,RouteWorkStep *out);
/* Only handles read-only show/export and RAM set/reset. Caller owns vehicle lock. */
void RouteLibrary_Command(const char *command);
#endif
