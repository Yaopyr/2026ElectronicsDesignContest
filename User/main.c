#include "stm32f10x.h"
#include "Delay.h"
#include "Tracking.h"
#include "Motor.h"
#include "OLED.h"
#include "UART.h"
#include "Stepper.h"

#include "pid.h"

#define BASE_SPEED	200//270
#define MAX_DIFF	80//160
#define POWER		2

#define PID_KP		0.08
#define PID_KI		0.015//0.02
#define PID_KD		0.015//0.008

volatile uint32_t g_elapsed_ms;

typedef enum {
	STATE_WAIT,
	STATE_RUN,
	STATE_DONE
} State_TypeDef;

typedef enum {
	MODE_02,	// 循迹 + 平衡球
	MODE_03,	// 定点测试：+340 → -340
	MODE_04		// 循迹3秒停止
} Mode_TypeDef;

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

	/* PB0 模式按钮（IPU） */
	GPIO_InitTypeDef GPIO_InitStructure2;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure2);

	/* 状态机 */
	State_TypeDef state = STATE_WAIT;
	Mode_TypeDef mode = MODE_02;

	while (1)
	{
		uint8_t sensors;
		int16_t pos_sum;
		uint8_t cnt;
		static uint8_t pb0_prev = 1;

		switch (state)
		{
		case STATE_WAIT:
			/* 平衡球 */
			pidInput = (double)UART_GetBallPosition();
			PID_Compute(&pid);
			Stepper_SetTarget((int32_t)pidOutput);

			/* OLED：模式 + 状态 */
			OLED_ShowString(1, 1, "M:");
			OLED_ShowNum(1, 3, (mode == MODE_02) ? 2 : (mode == MODE_03) ? 3 : 4, 2);
			OLED_ShowString(2, 1, "P:");
			OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
			OLED_ShowString(3, 1, "T:");
			OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);

			/* PB0 模式切换（下降沿） */
			{
				uint8_t pb0_now = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
				if (pb0_prev == 1 && pb0_now == 0)
					mode = (mode == MODE_02) ? MODE_03 : (mode == MODE_03) ? MODE_04 : MODE_02;
				pb0_prev = pb0_now;
			}

			/* PB15 启动 */
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 0)
			{
				pidSetpoint = 0;
				PID_SetMode(&pid, _PID_MODE_AUTOMATIC);
				OLED_Clear();
				g_elapsed_ms = 0;
				state = STATE_RUN;
			}
			break;

		case STATE_RUN:
			if (mode == MODE_02)
			{
				/* 停止条件 */
				if (Tracking_GetBlackCount() >= 3 && g_elapsed_ms >= 18000)
				{
					Motor_Stop();
					OLED_Clear();
					OLED_ShowString(1, 1, "Time:");
					OLED_ShowNum(2, 1, g_elapsed_ms / 1000, 5);
					OLED_ShowString(2, 7, "s");
					state = STATE_DONE;
					break;
				}

				sensors = Tracking_Read();
				OLED_ShowString(1, 1, "T:");
				OLED_ShowNum(1, 3, g_elapsed_ms / 1000, 4);
				OLED_ShowString(1, 7, "s");

				/* 循迹 */
				pos_sum = 0;
				cnt = 0;
				if (sensors & 0x01) { pos_sum -= 7; cnt++; }
				if (sensors & 0x02) { pos_sum -= 6; cnt++; }
				if (sensors & 0x04) { pos_sum -= 5; cnt++; }
				if (sensors & 0x08) { pos_sum -= 2; cnt++; }
				if (sensors & 0x10) { pos_sum += 2; cnt++; }
				if (sensors & 0x20) { pos_sum += 5; cnt++; }
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

				/* 平衡球 */
				pidInput = (double)UART_GetBallPosition();
				PID_Compute(&pid);
				Stepper_SetTarget((int32_t)pidOutput);

				OLED_ShowString(2, 1, "P:");
				OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
				OLED_ShowString(3, 1, "T:");
				OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);
			}
			else if (mode == MODE_04) /* 循迹3秒停止 */
			{
				if (g_elapsed_ms >= 4500)
				{
					Motor_Stop();
					OLED_Clear();
					OLED_ShowString(1, 1, "Time:");
					OLED_ShowNum(2, 1, g_elapsed_ms / 1000, 5);
					OLED_ShowString(2, 7, "s");
					state = STATE_DONE;
					break;
				}

				sensors = Tracking_Read();
				OLED_ShowString(1, 1, "T:");
				OLED_ShowNum(1, 3, g_elapsed_ms / 1000, 4);
				OLED_ShowString(1, 7, "s");

				/* 循迹 */
				pos_sum = 0;
				cnt = 0;
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

				/* 平衡球 */
				pidInput = (double)UART_GetBallPosition();
				PID_Compute(&pid);
				Stepper_SetTarget((int32_t)pidOutput);

				OLED_ShowString(2, 1, "P:");
				OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
				OLED_ShowString(3, 1, "T:");
				OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);
			}
			else /* MODE_03 — 定点测试 +340 → -340 */
			{
				static int8_t  sub03 = 0;
				static uint8_t stable_cnt = 0;
				int32_t error;

				switch (sub03)
				{
				case 0:	/* 设定目标 +340 */
					pidSetpoint = 340;
					sub03 = 1;
					stable_cnt = 0;
					break;

				case 1:	/* 等待稳定在 +340±5 持续 1s */
					error = (int32_t)(pidInput - 340);
					if (error < 0) error = -error;
					if (error <= 40)
					{
						if (++stable_cnt >= 10  /*50*/  )
						{
							sub03 = 2;
							stable_cnt = 0;
						}
					}
					else
						stable_cnt = 0;
					break;

				case 2:	/* 设定目标 -340 */
					pidSetpoint = -340;
					sub03 = 3;
					stable_cnt = 0;
					break;

				case 3:	/* 等待稳定在 -340±5 */
					error = (int32_t)(pidInput + 340);
					if (error < 0) error = -error;
					if (error <= 50)
					{
						if (++stable_cnt >= 25)
						{
							Motor_Stop();
							OLED_Clear();
							OLED_ShowString(1, 1, "Time:");
							OLED_ShowNum(2, 1, g_elapsed_ms / 1000, 5);
							OLED_ShowString(2, 7, "s");
							pidSetpoint = -340;
							sub03 = 0;
							stable_cnt = 0;
							state = STATE_DONE;
						}
					}
					else
						stable_cnt = 0;
					break;
				}

				/* 平衡球 */
				pidInput = (double)UART_GetBallPosition();
				PID_Compute(&pid);
				Stepper_SetTarget((int32_t)pidOutput);

				OLED_ShowString(1, 1, "M:03");
				OLED_ShowString(2, 1, "P:");
				OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
				OLED_ShowString(3, 1, "T:");
				OLED_ShowSignedNum(3, 3, (int32_t)pidSetpoint, 4);
				OLED_ShowString(4, 1, "t:");
				OLED_ShowNum(4, 3, g_elapsed_ms / 1000, 4);
				OLED_ShowString(4, 7, "s");
			}
			break;

		case STATE_DONE:
			/* 平衡球 */
			pidInput = (double)UART_GetBallPosition();
			PID_Compute(&pid);
			Stepper_SetTarget((int32_t)pidOutput);

			/* 按 PB15 重新开始 */
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 0)
			{
				Motor_Enable();
				OLED_Clear();
				g_elapsed_ms = 0;
				state = STATE_RUN;
			}
			break;
		}

		Delay_ms(20);
	}
}
