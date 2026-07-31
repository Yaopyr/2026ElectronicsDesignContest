#ifndef __IMU_H
#define __IMU_H
#include "stm32f10x.h"

uint8_t IMU_Init(void);
uint8_t IMU_ReadGyroZ(float *dps);
float   IMU_GetGyroOffset(void);

#endif
