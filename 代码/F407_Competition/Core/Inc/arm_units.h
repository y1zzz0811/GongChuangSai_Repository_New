#ifndef ARM_UNITS_H
#define ARM_UNITS_H
#include <stdint.h>
typedef struct {
  uint32_t pulses,travel_milli;
  uint8_t positive_direction;
} ArmUnitProfile;
void ArmUnits_Init(void);
const ArmUnitProfile *ArmUnits_Get(uint8_t axis);
uint8_t ArmUnits_SetScale(uint8_t axis,uint32_t pulses,uint32_t travel_milli);
uint8_t ArmUnits_SetDirection(uint8_t axis,uint8_t direction);
uint8_t ArmUnits_ParseMilli(const char *text,uint32_t *value);
uint8_t ArmUnits_ToPulses(uint8_t axis,uint32_t travel_milli,uint32_t *pulses);
uint8_t ArmUnits_FromPulses(uint8_t axis,uint32_t pulses,uint32_t *travel_milli);
#endif
