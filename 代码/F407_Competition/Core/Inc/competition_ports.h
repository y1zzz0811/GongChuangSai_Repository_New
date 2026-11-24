#ifndef COMPETITION_PORTS_H
#define COMPETITION_PORTS_H
#include "mission.h"
/* Future calibrated task-adapter contracts; no stub reports success.
 * Manual claw/tray commands and PWM ramps exist in servo.h; angle/position
 * feedback is absent. A finished ramp is not proof of a successful grasp.
 * An adapter maps these future hardware services to MissionPorts, checks real
 * feedback, then registers with Mission_RegisterPorts(). Do not set SERVICE_ALL
 * until every required adapter is calibrated and tested. */
typedef enum { TARGET_QR,TARGET_RAW,TARGET_RING,TARGET_STACK,TARGET_OBSTACLE } VisionTarget;
typedef struct {
  uint32_t timestamp_ms,sequence;
  int32_t x_milli_mm,y_milli_mm,yaw_cdeg;
  uint8_t valid,color,ring;
} VisionObservation;
typedef enum { SERVO_GRIP_OPEN,SERVO_GRIP_CLOSE,SERVO_TRAY_SLOT } ServoAction;
typedef struct { uint8_t homed_mask,limit_mask,fault_mask; uint32_t timestamp_ms; } HomeStatus;
typedef struct { char task_code[16]; uint8_t grabs,places; MissionState state; } DisplayData;
typedef struct {
  PortResult (*vision_request)(VisionTarget,uint8_t color,uint8_t ring);
  PortResult (*vision_read)(VisionObservation *); /* reject stale/invalid samples */
  PortResult (*servo_begin)(ServoAction,uint8_t slot);
  PortResult (*servo_poll)(void);
  PortResult (*home_begin)(uint8_t axis_mask);
  PortResult (*home_read)(HomeStatus *);
  PortResult (*display_update)(const DisplayData *); /* real acknowledgement */
} CompetitionHardwarePorts;
/* Startup key: PE2 adapter -> Competition_RequestStart() after debounce in main.
 * PE2 and PE8..PE10 limits remain uninitialized. USART3 display, UART5 QR and
 * TIM4 PB6/PB7 PWM are configured; mission task adapters are still absent.
 * Missing callback = unavailable, never PORT_DONE. */
#endif
