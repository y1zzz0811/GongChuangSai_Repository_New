#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK=0, HAL_ERROR=1 } HAL_StatusTypeDef;
typedef struct { int id; uint8_t *rx; } UART_HandleTypeDef;
typedef struct { int id; } I2C_HandleTypeDef;
typedef struct { uint32_t Prescaler,Period; } TIM_Base_InitTypeDef;
typedef struct { TIM_Base_InitTypeDef Init; uint32_t compare[2],started,events; } TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_EVENTSOURCE_UPDATE 1U
void ServoTest_SetCompare(TIM_HandleTypeDef *,uint32_t,uint32_t);
#define __HAL_TIM_SET_COMPARE(t,c,v) ServoTest_SetCompare((t),(c),(v))
uint32_t HAL_RCC_GetPCLK1Freq(void);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t);
#define __DMB() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
