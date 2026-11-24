#ifndef ROUTE2_H
#define ROUTE2_H
#include <stdint.h>
void Route2_Init(void);
void Route2_RequestStart(uint8_t no_lift);
void Route2_Poll(void);
void Route2_Report(void);
void Route2_ReportDecision(uint8_t force);
void Route2_Cancel(void);
uint8_t Route2_IsPending(void);
uint8_t Route2_IsActive(void);
void Route2_Command(const char *command,uint8_t allow_config);
#endif
