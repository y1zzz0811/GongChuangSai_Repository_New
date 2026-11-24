#include "stm32f4xx_hal.h"
#include <assert.h>
#include "servo_config.h"
void ServoTest_SetCompare(TIM_HandleTypeDef *t,uint32_t channel,uint32_t us) {
  assert(t && (channel==TIM_CHANNEL_1 || channel==TIM_CHANNEL_2));
  /* Independent measured boundary: observe EVERY compare write, not only
   * the value after a command/poll, so a transient unsafe pulse also fails. */
  assert(!us || (channel==TIM_CHANNEL_1?(us>=SERVO_CLAW_OPEN_US && us<=SERVO_CLAW_HARD_MAX_US):(us>=500U && us<=2500U)));
  t->compare[channel==TIM_CHANNEL_1?0:1]=us;
}
uint32_t servo_test_pclk=42000000U;
int servo_test_fail_event=0,servo_test_fail_channel=-1;
uint32_t HAL_RCC_GetPCLK1Freq(void) { return servo_test_pclk; }
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *t,uint32_t event) {
  if (servo_test_fail_event) return HAL_ERROR;
  t->events|=event; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t channel) {
  if ((int)channel==servo_test_fail_channel) return HAL_ERROR;
  t->started|=channel==TIM_CHANNEL_1?1U:2U; return HAL_OK;
}
