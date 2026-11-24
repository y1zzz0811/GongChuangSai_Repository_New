/* Physical displacement conversion, using integer 0.001 degree/mm units.
 * No floating-point scanf/printf, no dynamic allocation, no flash writes. */
#include "arm_units.h"
#include "arm_config.h"
#include <stddef.h>
static ArmUnitProfile profiles[3];
void ArmUnits_Init(void) {
  static const ArmUnitProfile defaults[3]={
    {ARM_M5_CAL_PULSES,ARM_M5_CAL_TRAVEL_MILLI,ARM_M5_POS_DIR},
    {ARM_M6_CAL_PULSES,ARM_M6_CAL_TRAVEL_MILLI,ARM_M6_POS_DIR},
    {ARM_M7_CAL_PULSES,ARM_M7_CAL_TRAVEL_MILLI,ARM_M7_POS_DIR}
  };
  uint8_t i;
  for (i=0U;i<3U;++i) profiles[i]=defaults[i];
}
const ArmUnitProfile *ArmUnits_Get(uint8_t axis) { return axis<3U?&profiles[axis]:NULL; }
uint8_t ArmUnits_SetScale(uint8_t axis,uint32_t pulses,uint32_t travel_milli) {
  if (axis>=3U || !pulses || pulses>1000000000U || !travel_milli || travel_milli>1000000000U) return 0U;
  profiles[axis].pulses=pulses; profiles[axis].travel_milli=travel_milli; return 1U;
}
uint8_t ArmUnits_SetDirection(uint8_t axis,uint8_t direction) {
  if (axis>=3U || direction>1U) return 0U;
  profiles[axis].positive_direction=direction; return 1U;
}
uint8_t ArmUnits_ParseMilli(const char *text,uint32_t *value) {
  uint32_t whole=0U,fraction=0U,multiplier=100U; uint8_t fractional_digits=0U;
  if (!text || !value || *text<'0' || *text>'9') return 0U;
  while (*text>='0' && *text<='9') {
    uint32_t digit=(uint32_t)(*text++-'0');
    if (whole>100000U || (whole==100000U && digit>0U)) return 0U;
    whole=whole*10U+digit;
  }
  if (*text=='.') {
    ++text;
    while (*text>='0' && *text<='9') {
      if (++fractional_digits>3U) return 0U;
      fraction+=(uint32_t)(*text++-'0')*multiplier; multiplier/=10U;
    }
    if (!fractional_digits) return 0U;
  }
  if (*text || (whole==1000000U && fraction)) return 0U;
  *value=whole*1000U+fraction; return 1U;
}
static uint8_t RoundedRatio(uint32_t amount,uint32_t numerator,uint32_t denominator,uint32_t *out) {
  uint64_t result;
  if (!denominator || !numerator || !out) return 0U;
  result=((uint64_t)amount*numerator+denominator/2U)/denominator;
  if (result>UINT32_MAX) return 0U;
  *out=(uint32_t)result; return 1U;
}
uint8_t ArmUnits_ToPulses(uint8_t axis,uint32_t travel_milli,uint32_t *pulses) {
  if (axis>=3U) return 0U;
  return RoundedRatio(travel_milli,profiles[axis].pulses,profiles[axis].travel_milli,pulses);
}
uint8_t ArmUnits_FromPulses(uint8_t axis,uint32_t pulses,uint32_t *travel_milli) {
  if (axis>=3U) return 0U;
  return RoundedRatio(pulses,profiles[axis].travel_milli,profiles[axis].pulses,travel_milli);
}
