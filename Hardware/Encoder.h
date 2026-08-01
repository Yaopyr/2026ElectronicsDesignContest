#ifndef __ENCODER_H
#define __ENCODER_H
/* 编码器接口（详见 Encoder.c 文件头：TIM3/TIM4 引脚、4 倍频、20ms 增量约定） */
#include "stm32f10x.h"

void Encoder_Init(void);
int16_t Encoder_GetLeft(void);
int16_t Encoder_GetRight(void);
int16_t Encoder_GetSpeedLeft(void);
int16_t Encoder_GetSpeedRight(void);
void Encoder_Update(void);

#endif
