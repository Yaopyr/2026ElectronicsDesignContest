#ifndef __TRACKING_H
#define __TRACKING_H
#include "stm32f10x.h"

extern volatile uint8_t g_lt_raw;
extern volatile float  g_lt_error;
extern volatile uint8_t g_lt_ok;

void    Tracking_Init(void);
uint8_t Tracking_Read(void);
uint8_t Tracking_GetBlackCount(void);
float   Tracking_GetError(void);
#endif
