#include "JY61P.h"
#include "Delay.h"

extern volatile uint32_t g_elapsed_ms;

#define JY61P_WZ_SCALE  16.384f	/* int16 ±32768 → ±2000°/s */

static volatile int16_t  s_wz = 0;	/* 最新 Z 轴角速度（原始 int16） */
static volatile uint32_t s_wz_time = 0;	/* 收到帧的 g_elapsed_ms 时间戳 */

static void JY61P_Send(const uint8_t *buf, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++) {
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) ;
		USART_SendData(USART2, buf[i]);
	}
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) ;
}

/**
  * @brief  USART2 初始化：PA2=TX, PA3=RX, 9600-8N1, RX中断
  *         JY61P 实际波特率 9600（逻辑分析仪实测），主动输出
  */
uint8_t JY61P_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* PA2=TX 复用推挽（发配置指令用）+ PA3=RX 浮空输入 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART2: 9600-8N1（模块当前波特率，随后切到 115200） */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	/* RXNE 中断使能 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);

	/* 1) 9600 下：解锁 + 切波特率 115200（BAUD=0x06，官方文档） */
	uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};	/* 解锁 */
	uint8_t baud[]   = {0xFF, 0xAA, 0x04, 0x06, 0x00};	/* BAUD=0x06 → 115200 */
	JY61P_Send(unlock, sizeof(unlock));	Delay_ms(20);
	JY61P_Send(baud, sizeof(baud));		Delay_ms(20);

	/* 2) 模块已切 115200 → 重配 USART2 波特率 */
	USART_Cmd(USART2, DISABLE);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
	Delay_ms(20);

	/* 3) 115200 下：回传速率 100Hz（RRATE=0x09，官方文档）+ 保存 */
	uint8_t rate[] = {0xFF, 0xAA, 0x03, 0x09, 0x00};	/* RRATE=0x09 → 100Hz */
	uint8_t save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};	/* 保存 */
	JY61P_Send(unlock, sizeof(unlock));	Delay_ms(20);
	JY61P_Send(rate, sizeof(rate));		Delay_ms(20);
	JY61P_Send(save, sizeof(save));		Delay_ms(50);

	return 1;	/* 串口初始化不探测硬件，恒成功 */
}

/**
  * @brief  解析 JY61P 0x52 角速度帧（维特协议，11 字节，逻辑分析仪实测）：
  *         55 52 wxl wxh wyl wyh wzl wzh TMPL TMPH SUM
  *         校验和 = 前 10 字节累加低 8 位
  */
void USART2_IRQHandler(void)
{
	static uint8_t buf[11];
	static uint8_t idx = 0;

	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uint8_t c = USART_ReceiveData(USART2);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

		if (idx == 0) {
			if (c == 0x55) buf[idx++] = c;
		} else if (idx == 1) {
			if (c == 0x52) buf[idx++] = c;
			else idx = 0;
		} else {
			buf[idx++] = c;
			if (idx >= 11) {
				uint8_t sum = 0;
				for (uint8_t i = 0; i < 10; i++)
					sum += buf[i];
				if (sum == buf[10]) {
					s_wz = (int16_t)((buf[7] << 8) | buf[6]);
					s_wz_time = g_elapsed_ms;
				}
				idx = 0;
			}
		}
	}
}

/**
  * @brief  获取 Z 轴角速度（dps）
  * @param  dps 输出：°/s
  * @retval 1=有新鲜数据（≤300ms），0=断连/超时
  */
uint8_t JY61P_GetWz(float *dps)
{
	if ((uint32_t)(g_elapsed_ms - s_wz_time) > 300)
		return 0;

	*dps = (float)s_wz / JY61P_WZ_SCALE;
	return 1;
}
