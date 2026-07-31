#include "stm32f10x.h"

/* TB6612 引脚定义 */
#define AIN1_PORT	GPIOA
#define AIN1_PIN	GPIO_Pin_12
#define AIN2_PORT	GPIOB
#define AIN2_PIN	GPIO_Pin_12
#define BIN1_PORT	GPIOB
#define BIN1_PIN	GPIO_Pin_13
#define BIN2_PORT	GPIOB
#define BIN2_PIN	GPIO_Pin_14
#define STBY_PORT	GPIOB
#define STBY_PIN	GPIO_Pin_5

#define PWM_PERIOD	999	// ARR: 72M/72/1000 = 1kHz, 分辨率1000

/**
  * 函    数：电机驱动初始化
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Init(void)
{
	/* 开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* 方向引脚 + STBY 推挽输出 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = AIN1_PIN;
	GPIO_Init(AIN1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AIN2_PIN;
	GPIO_Init(AIN2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BIN1_PIN;
	GPIO_Init(BIN1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BIN2_PIN;
	GPIO_Init(BIN2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = STBY_PIN;
	GPIO_Init(STBY_PORT, &GPIO_InitStructure);

	/* PWM引脚 PA8(TIM1_CH1) + PA11(TIM1_CH4) 复用推挽输出 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* TIM1 时基 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;	// 72MHz / 72 = 1MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* TIM1_CH1 PWM 输出 */
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	/* TIM1_CH4 PWM 输出 */
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	/* TIM1 主输出使能 + 自动重载预装载 */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);

	/* 初始状态：STBY拉高使能，电机停转 */
	GPIO_SetBits(STBY_PORT, STBY_PIN);
	GPIO_ResetBits(AIN1_PORT, AIN1_PIN);
	GPIO_ResetBits(AIN2_PORT, AIN2_PIN);
	GPIO_ResetBits(BIN1_PORT, BIN1_PIN);
	GPIO_ResetBits(BIN2_PORT, BIN2_PIN);
}

/**
  * 函    数：设置左电机速度
  * 参    数：speed 速度值，范围 -999 ~ 999，正=前进，负=后退
  * 返 回 值：无
  */
void Motor_SetLeft(int16_t speed)
{
	// ponytail: naive clamping, fine for basic tracking
	if (speed >  PWM_PERIOD) speed =  PWM_PERIOD;
	if (speed < -PWM_PERIOD) speed = -PWM_PERIOD;

	if (speed > 0)
	{
		GPIO_SetBits(AIN1_PORT, AIN1_PIN);
		GPIO_ResetBits(AIN2_PORT, AIN2_PIN);
		TIM_SetCompare1(TIM1, speed);
	}
	else if (speed < 0)
	{
		GPIO_ResetBits(AIN1_PORT, AIN1_PIN);
		GPIO_SetBits(AIN2_PORT, AIN2_PIN);
		TIM_SetCompare1(TIM1, -speed);
	}
	else
	{
		GPIO_ResetBits(AIN1_PORT, AIN1_PIN);
		GPIO_ResetBits(AIN2_PORT, AIN2_PIN);
		TIM_SetCompare1(TIM1, 0);
	}
}

/**
  * 函    数：设置右电机速度
  * 参    数：speed 速度值，范围 -999 ~ 999，正=前进，负=后退
  * 返 回 值：无
  */
void Motor_SetRight(int16_t speed)
{
	if (speed >  PWM_PERIOD) speed =  PWM_PERIOD;
	if (speed < -PWM_PERIOD) speed = -PWM_PERIOD;

	if (speed > 0)
	{
		GPIO_SetBits(BIN1_PORT, BIN1_PIN);
		GPIO_ResetBits(BIN2_PORT, BIN2_PIN);
		TIM_SetCompare4(TIM1, speed);
	}
	else if (speed < 0)
	{
		GPIO_ResetBits(BIN1_PORT, BIN1_PIN);
		GPIO_SetBits(BIN2_PORT, BIN2_PIN);
		TIM_SetCompare4(TIM1, -speed);
	}
	else
	{
		GPIO_ResetBits(BIN1_PORT, BIN1_PIN);
		GPIO_ResetBits(BIN2_PORT, BIN2_PIN);
		TIM_SetCompare4(TIM1, 0);
	}
}

/**
  * 函    数：电机制动停止
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Stop(void)
{
	/* 主动短路刹车：PWM清零 + 方向引脚全高（IN1=IN2=HIGH → 低侧FET导通，电机端子短接制动）*/
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare4(TIM1, 0);
	GPIO_SetBits(AIN1_PORT, AIN1_PIN);
	GPIO_SetBits(AIN2_PORT, AIN2_PIN);
	GPIO_SetBits(BIN1_PORT, BIN1_PIN);
	GPIO_SetBits(BIN2_PORT, BIN2_PIN);
	GPIO_ResetBits(STBY_PORT, STBY_PIN);	// 双重保险
}

void Motor_Enable(void)
{
	GPIO_SetBits(STBY_PORT, STBY_PIN);
}
