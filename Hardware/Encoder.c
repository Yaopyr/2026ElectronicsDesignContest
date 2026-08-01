#include "Encoder.h"

/* ============================================================================
 * Encoder.c — 正交编码器计数（TIM 编码器模式，TI12 双沿 → 4 倍频）
 *   右编码器：TIM3  CH1=PA6  CH2=PA7（默认引脚）
 *   左编码器：TIM4  CH1=PD12 CH2=PD13（GPIO_Remap_TIM4 重映射，默认 PB6/PB7）
 *   计数：ARR=0xFFFF 自由运行，按 int16 读取得自然回绕，差值不受溢出影响
 *   速度：Encoder_Update() 每次主循环(20ms)调用，speed = 当前计数 − 上次计数
 *         （即 20ms 内的脉冲增量，正值=正转）
 * ========================================================================== */

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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
	                           TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetAutoreload(TIM3, 0xFFFF);
	TIM_Cmd(TIM3, ENABLE);

	/* 左编码器 TIM4 (PD12=CH1, PD13=CH2)，需重映射（默认 PB6/PB7） */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

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

/* 刷新速度：主循环每次(20ms)调用，计算两轮本周期增量 */
void Encoder_Update(void)
{
	int16_t cur_left  = Encoder_GetLeft();
	int16_t cur_right = Encoder_GetRight();

	speed_left  = cur_left  - last_left;
	speed_right = cur_right - last_right;

	last_left  = cur_left;
	last_right = cur_right;
}
