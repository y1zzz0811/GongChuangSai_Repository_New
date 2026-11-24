#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H
#include "stm32f4xx_hal.h"
#include <stddef.h>
void AppConsole_Init(UART_HandleTypeDef *);
void AppConsole_Poll(void);
uint8_t AppConsole_StopPending(void);
void AppConsole_ClearInput(void);
uint8_t AppConsole_Telemetry(void);
void AppConsole_SetTelemetry(uint8_t enabled);
void App_Print(const char *);
void App_Log(const char *,...);
/* Bounded integer formatter: %s, %c, %u/%d/%x/%X, optional l and zero width.
 * Returns would-have-written count, matching snprintf. No float support. */
int App_Format(char *,size_t,const char *,...);
int App_ParseU32(const char *,uint32_t low,uint32_t high,uint32_t *out);
int App_ParseHexByte(const char *,uint8_t *out);
#endif
