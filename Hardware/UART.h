#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"

void UART_Init(void);
int16_t UART_GetBallPosition(void);	// 返回最新小球位置，无新数据返回上次值

#endif
