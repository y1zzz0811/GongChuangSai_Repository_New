#ifndef MISSION_H
#define MISSION_H
#include <stdint.h>
/* Future integrations report real feedback, never timer-based assumed success. */
typedef enum { PORT_ERROR=-1,PORT_WAIT=0,PORT_DONE=1 } PortResult;
typedef enum { GO_QR,READ_QR,GO_RAW,LOAD_RAW,GO_ROUGH,PLACE_ROUGH,RELOAD_ROUGH,GO_TEMP,PLACE_TEMP,GO_HOME } MissionOp;
typedef enum { MISSION_IDLE,MISSION_RUNNING,MISSION_DONE,MISSION_STOPPED,MISSION_FAULT } MissionState;
enum { SERVICE_VISION=1U,SERVICE_SERVO=2U,SERVICE_HOME=4U,SERVICE_DISPLAY=8U,SERVICE_NAV=16U,SERVICE_HANDLING=32U,SERVICE_ALL=63U };
typedef struct {
  MissionOp op;
  uint8_t batch,index,color,ring,stack,home;
} MissionRequest;
typedef struct {
  /* ready(): include freshness, homing, calibration and device health checks. */
  uint32_t (*ready)(void);
  PortResult (*begin)(const MissionRequest *);
  PortResult (*poll)(const MissionRequest *);
  void (*cancel)(void);
} MissionPorts;
typedef struct { uint8_t colors[2][3],rings[2][3],temp_ring[7]; char text[16]; } MissionCode;
typedef struct {
  MissionState state;
  uint8_t home,grabs,places;
  uint32_t elapsed_ms;
  char task_code[16];
} MissionSnapshot;
void Mission_Init(void);
/* Register only while not running. Null registration disconnects all services. */
uint8_t Mission_RegisterPorts(const MissionPorts *);
uint8_t Mission_ParseCode(const char *,MissionCode *);
/* Called by real QR service only in READ_QR. Rejects malformed/new late codes. */
uint8_t Mission_AcceptCode(const char *);
uint8_t Mission_Start(void);
/* Explicit commissioning time limits, with a separate budget per request. */
uint8_t Mission_StartCommissioning(uint32_t total_ms,uint32_t request_ms);
void Mission_Poll(void);
void Mission_Stop(uint8_t fault);
void Mission_Reset(void);
uint8_t Mission_SetHome(uint8_t home);
MissionState Mission_GetState(void);
uint8_t Mission_IsRunning(void);
void Mission_ShowStatus(void);
void Mission_GetSnapshot(MissionSnapshot *out);
#endif
