#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f10x.h"

#define STEP_MAX_SPEED	2000	// 最大步频 (Hz)
#define STEP_RANGE		100		// ±45° = ±100步 (1.8°, 4细分, 800脉冲/圈)

void Stepper_Init(void);
void Stepper_SetTarget(int32_t steps);	// 设置目标位置（相对中位），正=抬升

#endif
