#ifndef COMPETITION_H
#define COMPETITION_H
#include "stm32f4xx_hal.h"
void Competition_Init(UART_HandleTypeDef *,UART_HandleTypeDef *,UART_HandleTypeDef *);
void Competition_Poll(void);
void Competition_Command(const char *);
void Competition_Stop(const char *,uint8_t);
/* Future PE2 button adapter calls this once per debounced press in main loop. */
uint8_t Competition_RequestStart(void);
#endif
