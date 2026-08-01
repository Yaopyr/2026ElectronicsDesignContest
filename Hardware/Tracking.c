#include "Tracking.h"
#include "Delay.h"

/* ============================================================================
 * Tracking.c — 8 路数字循迹传感器（硬件 I2C2）
 *   接线：SCL=PB10, SDA=PB11（复用开漏），7 位从机地址 0x4C
 *   协议：发送命令 0xDD 进入数字模式；此后每次读回 1 字节，
 *         每 bit 对应一路传感器，黑线 = 0（低电平有效）
 *   约定：读失败时置 g_lt_raw=0xFF（全丢线哨兵）→ 主循环连续 5 次急停
 *         sensor_pos 为各路距中线物理位置(mm)，用于加权质心误差
 *   注意：Init 时关闭 JTAG 仅留 SWD（GPIO_Remap_SWJ_JTAGDisable），
 *         释放 PB3/PB4/PB15 供其他外设使用（全局生效）
 * ========================================================================== */

#define LT_I2C_ADDR        0x4C
#define LT_CMD_DIGITAL     0xDD
#define LT_WRITE           ((LT_I2C_ADDR) << 1)
#define LT_READ            (((LT_I2C_ADDR) << 1) | 0x01)

/* 8 路传感器距中线位置(mm)，负=左、正=右；下方注释保留各次调测的历史位置组 */
static const float sensor_pos[8] = {
//	-42.0f, -30.0f, -18.0f, -6.0f, 6.0f, 18.0f, 30.0f, 42.0f
//	-42.0f, -15.0f, -9.0f, -3.0f, 3.0f, 9.0f, 15.0f, 42.0f
//	-42.0f, -30.0f, -6.0f, -1.0f, 1.0f, 6.0f, 30.0f, 42.0f
//	-42.0f, -30.0f, -3.0f, -0.9f, 0.9f, 3.0f, 30.0f, 42.0f
//	-48.0f, -35.0f, -1.0f, -0.8f, 0.8f, 1.0f, 35.0f, 48.0f
//		-50.0f, -35.0f, -1.0f, -0.8f, 0.8f, 1.0f, 35.0f, 50.0f
		-50.0f, -38.0f, -1.0f, -0.8f, 0.8f, 1.0f, 38.0f, 50.0f	/* 当前使用组 */



};

volatile uint8_t g_lt_raw  = 0;
volatile float  g_lt_error = 0.0f;
volatile uint8_t g_lt_ok   = 0;	/* 最近一次I2C读成功标志 */
static float last_error = 0.0f;

/* ---- I2C2 硬件驱动（PB10=SCL, PB11=SDA） ---- */

static void I2C2_InitPins(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);
}

/* 读 1 字节：写命令 0xDD → 重复起始 → 读 1 字节（NACK+STOP 收尾）。
 * 每步均带 10000 次轮询超时，超时即发 STOP 并清 AF/BERR/ARLO 标志，防止总线死锁 */
static uint8_t I2C2_ReadByte(uint8_t *val)
{
	uint32_t timeout;

	/* 写命令 0xDD */
	I2C_GenerateSTART(I2C2, ENABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C2, ENABLE); I2C_ClearFlag(I2C2, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO); return 0; }
	}
	I2C_Send7bitAddress(I2C2, LT_WRITE, I2C_Direction_Transmitter);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C2, ENABLE); I2C_ClearFlag(I2C2, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO); return 0; }
	}
	I2C_SendData(I2C2, LT_CMD_DIGITAL);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C2, ENABLE); I2C_ClearFlag(I2C2, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO); return 0; }
	}

	/* 重复起始 + 读 1 字节 */
	I2C_GenerateSTART(I2C2, ENABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C2, ENABLE); I2C_ClearFlag(I2C2, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO); return 0; }
	}
	I2C_Send7bitAddress(I2C2, LT_READ, I2C_Direction_Receiver);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C2, ENABLE); I2C_ClearFlag(I2C2, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO); return 0; }
	}

	I2C_AcknowledgeConfig(I2C2, DISABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--timeout == 0) { I2C_AcknowledgeConfig(I2C2, ENABLE); I2C_ClearFlag(I2C2, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO); return 0; }
	}
	*val = I2C_ReceiveData(I2C2);

	I2C_GenerateSTOP(I2C2, ENABLE);
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	return 1;
}

/* ---- 公开接口 ---- */

void Tracking_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	I2C2_InitPins();
	Delay_ms(100);	/* 等传感器模块上电就绪 */

	/* 启动传感器数字模式 */
	I2C_GenerateSTART(I2C2, ENABLE);
	uint32_t timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) break;
	}
	I2C_Send7bitAddress(I2C2, LT_WRITE, I2C_Direction_Transmitter);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (--timeout == 0) break;
	}
	I2C_SendData(I2C2, LT_CMD_DIGITAL);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (--timeout == 0) break;
	}
	I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t Tracking_Read(void)
{
	uint8_t raw;
	if (!I2C2_ReadByte(&raw)) {
		g_lt_ok = 0;
		g_lt_raw = 0xFF;	/* 读失败按全丢线处理，触发急停保护 */
		return 0x00;
	}
	g_lt_ok = 1;
	g_lt_raw = raw;
	return ~raw;  /* 反相：bit=1 表示黑线，兼容旧接口 */
}

uint8_t Tracking_GetBlackCount(void)
{
	uint8_t raw = g_lt_raw;
	uint8_t cnt = 0;
	for (int i = 0; i < 8; i++) {
		if (!(raw & (1 << i)))
			cnt++;
	}
	return cnt;
}

float Tracking_GetError(void)
{
	uint8_t raw;
	if (!I2C2_ReadByte(&raw)) {
		g_lt_ok = 0;
		g_lt_raw = 0xFF;	/* 读失败按全丢线处理，触发急停保护 */
		return last_error;
	}
	g_lt_ok = 1;
	g_lt_raw = raw;

	float sum = 0.0f;
	uint8_t cnt = 0;

	for (int i = 0; i < 8; i++) {
		if (!(raw & (1 << (7 - i)))) {
			sum += sensor_pos[i];
			cnt++;
		}
	}

	if (cnt == 0)
		return last_error;

	last_error = sum / cnt;
	g_lt_error = last_error;
	return last_error;
}
