#include "stm32f10x.h"
#include "Delay.h"
#include "Tracking.h"
#include "Motor.h"
#include "OLED.h"
#include "UART.h"
#include "Stepper.h"

#include "pid.h"

#define BASE_SPEED	270
#define MAX_DIFF	160
#define POWER		2

#define PID_KP		0.08
#define PID_KI		0.02
#define PID_KD		0.008

volatile uint32_t g_elapsed_ms;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	OLED_Init();
	Tracking_Init();
	Motor_Init();
	UART_Init();
	Stepper_Init();
	OLED_ShowString(1, 1, "Ready!");

	/* TIM2 1ms */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM2, ENABLE);

	/* PID */
	double pidInput = 0, pidOutput = 0, pidSetpoint = 0;
	PID_TypeDef pid;
	PID(&pid, &pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD,
	    _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&pid, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&pid, 20);
	PID_SetOutputLimits(&pid, -STEP_RANGE, STEP_RANGE);

	/* 阶段1：上电即运行平衡球，等待 PB15 启动信号 */
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 1)
	{
		pidInput = (double)UART_GetBallPosition();
		PID_Compute(&pid);
		Stepper_SetTarget((int32_t)pidOutput);

		OLED_ShowString(2, 1, "P:");
		OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
		OLED_ShowString(3, 1, "T:");
		OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);

		Delay_ms(20);
	}

	/* 阶段2：启动信号触发，循迹 + 平衡球 */
	OLED_Clear();
	Delay_ms(50);

	while (1)
	{
		uint8_t blackCount = Tracking_GetBlackCount();

		if (blackCount >= 3)
		{
			Motor_Stop();
			OLED_Clear();
			OLED_ShowString(1, 1, "Time:");
			OLED_ShowNum(2, 1, g_elapsed_ms / 1000, 5);
			OLED_ShowString(2, 7, "s");
			while (1);
		}

		uint8_t sensors = Tracking_Read();
		OLED_ShowBinNum(1, 1, sensors, 8);

		/* 循迹 */
		int16_t pos_sum = 0;
		uint8_t cnt = 0;
		if (sensors & 0x01) { pos_sum -= 7; cnt++; }
		if (sensors & 0x02) { pos_sum -= 6; cnt++; }
		if (sensors & 0x04) { pos_sum -= 4; cnt++; }
		if (sensors & 0x08) { pos_sum -= 2; cnt++; }
		if (sensors & 0x10) { pos_sum += 2; cnt++; }
		if (sensors & 0x20) { pos_sum += 4; cnt++; }
		if (sensors & 0x40) { pos_sum += 6; cnt++; }
		if (sensors & 0x80) { pos_sum += 7; cnt++; }

		if (cnt > 0)
		{
			int16_t pos = pos_sum / cnt;
			uint16_t abs_pos = (pos > 0) ? pos : -pos;
			uint16_t numerator = MAX_DIFF;
			for (uint8_t p = 0; p < POWER; p++)
				numerator = numerator * abs_pos / 7;
			int16_t diff = numerator;

			if (pos < 0)
			{
				Motor_SetLeft( BASE_SPEED - diff);
				Motor_SetRight(BASE_SPEED + diff);
			}
			else
			{
				Motor_SetLeft( BASE_SPEED + diff);
				Motor_SetRight(BASE_SPEED - diff);
			}
		}

		/* 小球平衡 */
		pidInput = (double)UART_GetBallPosition();
		PID_Compute(&pid);
		Stepper_SetTarget((int32_t)pidOutput);

		/* 调试 */
		OLED_ShowString(2, 1, "P:");
		OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
		OLED_ShowString(3, 1, "T:");
		OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);

		Delay_ms(20);
	}
}
