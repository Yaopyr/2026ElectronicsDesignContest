#ifndef __MOTOR_H
#define __MOTOR_H

/* TB6612 电机驱动接口（详见 Motor.c 文件头：引脚/TIM 通道/速度约定） */
#include "stm32f10x.h"

void Motor_Init(void);
void Motor_SetLeft(int16_t speed);
void Motor_SetRight(int16_t speed);
void Motor_Stop(void);

#endif
