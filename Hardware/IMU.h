#ifndef __IMU_H
#define __IMU_H
/* ⚠ 死代码：ICM20948 驱动，主程序已改用 JY61P，本模块仅保留参考 */
#include "stm32f10x.h"

uint8_t IMU_Init(void);
uint8_t IMU_ReadGyroZ(float *dps);
float   IMU_GetGyroOffset(void);

#endif
