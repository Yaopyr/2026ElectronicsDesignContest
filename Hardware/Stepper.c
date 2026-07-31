#include "Stepper.h"

/* 引脚：PB8=STEP, PB9=DIR */
#define STEP_PORT	GPIOB
#define STEP_PIN	GPIO_Pin_8
#define DIR_PORT	GPIOB
#define DIR_PIN		GPIO_Pin_9

volatile int32_t g_step_target  = 0;	// 目标位置（步数）
volatile int32_t g_step_current = 0;	// 当前位置

/**
  * @brief  步进电机初始化：GPIO + TIM4 脉冲生成
  *          TIM4 100µs (10kHz) 中断，每 500µs 一个脉冲 → 2kHz 步频
  */
void Stepper_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* STEP/DIR/EN 推挽输出 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = STEP_PIN | DIR_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 初始状态 */
	GPIO_ResetBits(STEP_PORT, STEP_PIN);
	GPIO_ResetBits(DIR_PORT, DIR_PIN);

	/* TIM4 时基：72MHz / 72 = 1MHz, 周期100 → 10kHz (100µs) */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 100 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

void Stepper_SetTarget(int32_t steps)
{
	/* 限幅 */
	if (steps >  STEP_RANGE) steps =  STEP_RANGE;
	if (steps < -STEP_RANGE) steps = -STEP_RANGE;
	g_step_target = steps;
}

/**
  * @brief  TIM4 中断：步进脉冲生成（两相法）
  *          100µs * 5 = 500µs 一个完整脉冲 → 2kHz 步频
  */
void TIM4_IRQHandler(void)
{
	static uint8_t pulse_tick = 0;

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		if (g_step_current == g_step_target)
			return;

		pulse_tick++;
		if (pulse_tick < 5)	// 每 5 个 tick (500µs) 一步
			return;
		pulse_tick = 0;

		if (g_step_current < g_step_target)
		{
			GPIO_SetBits(DIR_PORT, DIR_PIN);
			g_step_current++;
		}
		else
		{
			GPIO_ResetBits(DIR_PORT, DIR_PIN);
			g_step_current--;
		}

		/* STEP 脉冲 */
		GPIO_SetBits(STEP_PORT, STEP_PIN);
		for (volatile uint8_t i = 0; i < 20; i++) __NOP();	// ~1µs
		GPIO_ResetBits(STEP_PORT, STEP_PIN);
	}
}
