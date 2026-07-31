#include "Encoder.h"

static int16_t last_left  = 0;
static int16_t last_right = 0;
static int16_t speed_left  = 0;
static int16_t speed_right = 0;

void Encoder_Init(void)
{
	/* 右编码器 TIM3 (PA6=CH1, PA7=CH2) */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
	                           TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetAutoreload(TIM3, 0xFFFF);
	TIM_Cmd(TIM3, ENABLE);

	/* 左编码器 TIM4 (PD12=CH1, PD13=CH2) */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
	                           TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetAutoreload(TIM4, 0xFFFF);
	TIM_Cmd(TIM4, ENABLE);
}

int16_t Encoder_GetLeft(void)
{
	return (int16_t)TIM_GetCounter(TIM4);
}

int16_t Encoder_GetRight(void)
{
	return (int16_t)TIM_GetCounter(TIM3);
}

int16_t Encoder_GetSpeedLeft(void)
{
	return speed_left;
}

int16_t Encoder_GetSpeedRight(void)
{
	return speed_right;
}

void Encoder_Update(void)
{
	int16_t cur_left  = Encoder_GetLeft();
	int16_t cur_right = Encoder_GetRight();

	speed_left  = cur_left  - last_left;
	speed_right = cur_right - last_right;

	last_left  = cur_left;
	last_right = cur_right;
}
