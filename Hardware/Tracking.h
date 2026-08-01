#ifndef __TRACKING_H
#define __TRACKING_H
/* 8 路循迹传感器接口（详见 Tracking.c 文件头：I2C2/协议/丢线约定）
 *   g_lt_raw   原始 8bit（黑线=0），读失败为 0xFF 哨兵
 *   g_lt_ok    最近一次 I2C 读成功标志（1=OK）
 *   g_lt_error 最近一次质心误差(mm)（调试用） */
#include "stm32f10x.h"

extern volatile uint8_t g_lt_raw;
extern volatile float  g_lt_error;
extern volatile uint8_t g_lt_ok;

void    Tracking_Init(void);
uint8_t Tracking_Read(void);
uint8_t Tracking_GetBlackCount(void);
float   Tracking_GetError(void);
#endif
