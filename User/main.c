#include "stm32f10x.h"
#include "Delay.h"
#include "Tracking.h"
#include "Motor.h"
#include "OLED.h"

#define BASE_SPEED	270		// 直行基础速度 (0~999)
#define MAX_DIFF	160		// 边缘最大差速（最外侧传感器时）
#define POWER		2		// 幂次：2=平方，越大中心越平缓、边缘越陡

volatile uint32_t g_elapsed_ms;	// TIM2中断累计毫秒

int main(void)
{
	/* 模块初始化 */
	OLED_Init();
	Tracking_Init();
	Motor_Init();
	OLED_ShowString(1, 1, "Ready!");

	/* 等待启动信号：PB15 低电平 */
	Tracking_WaitStart();
	Delay_ms(50);	// 消抖

	/* TIM2 1ms 计时：72MHz / 72 = 1MHz, 周期1000 → 1kHz */
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

	while (1)
	{
		uint8_t blackCount = Tracking_GetBlackCount();

		/* 3个及以上传感器检测到黑线 → 停车 */
		if (blackCount >= 3)
		{
			Motor_Stop();
			OLED_Clear();
			OLED_ShowString(1, 1, "Time:");
			OLED_ShowNum(2, 1, g_elapsed_ms / 1000, 5);
			OLED_ShowString(2, 7, "s");
			while (1);	// 停住不动
		}

		uint8_t sensors = Tracking_Read();

		/* OLED显示传感器状态 */
		OLED_ShowBinNum(1, 1, sensors, 8);

		/* 位置加权平均：S0=-7, S1=-5, ..., S7=+7 */
		int16_t pos_sum = 0;
		uint8_t cnt = 0;
		if (sensors & 0x01) { pos_sum -= 7; cnt++; }	// S0
		if (sensors & 0x02) { pos_sum -= 6; cnt++; }	// S1
		if (sensors & 0x04) { pos_sum -= 4; cnt++; }	// S2
		if (sensors & 0x08) { pos_sum -= 2; cnt++; }	// S3
		if (sensors & 0x10) { pos_sum += 2; cnt++; }	// S4
		if (sensors & 0x20) { pos_sum += 4; cnt++; }	// S5
		if (sensors & 0x40) { pos_sum += 6; cnt++; }	// S6
		if (sensors & 0x80) { pos_sum += 7; cnt++; }	// S7

		int16_t diff = 0;
		if (cnt > 0)
		{
			int16_t pos = pos_sum / cnt;		// 加权平均位置 (-7 ~ +7)
			/* 幂次曲线：diff = MAX_DIFF * (|pos|/7)^POWER */
			uint16_t abs_pos = (pos > 0) ? pos : -pos;
			uint16_t numerator = MAX_DIFF;
			for (uint8_t p = 0; p < POWER; p++)
				numerator = numerator * abs_pos / 7;
			diff = numerator;

			if (pos < 0)
			{
				/* 黑线偏左 → 左转 */
				Motor_SetLeft( BASE_SPEED - diff);
				Motor_SetRight(BASE_SPEED + diff);
			}
			else
			{
				/* 黑线偏右 → 右转 */
				Motor_SetLeft( BASE_SPEED + diff);
				Motor_SetRight(BASE_SPEED - diff);
			}
		}

		Delay_ms(20);
	}
}
