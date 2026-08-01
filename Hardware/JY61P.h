#ifndef __JY61P_H
#define __JY61P_H
/* JY61P 六轴陀螺仪接口（USART2，详见 JY61P.c 文件头：协议/配置命令） */
#include "stm32f10x.h"

uint8_t  JY61P_Init(void);
uint8_t  JY61P_GetWz(float *dps);

#endif
