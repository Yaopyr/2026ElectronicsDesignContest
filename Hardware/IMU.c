#include "IMU.h"
#include "Delay.h"

#define ICM20948_ADDR       0x68
#define ICM20948_WRITE      (ICM20948_ADDR << 1)
#define ICM20948_READ       ((ICM20948_ADDR << 1) | 0x01)

/* 寄存器 */
#define REG_WHO_AM_I        0x00
#define REG_PWR_MGMT_1      0x06
#define REG_GYRO_CONFIG_1   0x1B
#define REG_GYRO_ZOUT_H     0x47
#define REG_GYRO_ZOUT_L     0x48

#define GYRO_SCALE          65.5f    /* ±500dps */

static float gyro_offset = 0.0f;

/* ---- I2C 总线恢复（软件 bit-bang，解除从机锁定） ---- */
static void I2C1_BusRecovery(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_6);   /* SCL = 1 */
	GPIO_SetBits(GPIOB, GPIO_Pin_7);   /* SDA = 1 */
	Delay_ms(10);

	/* 发送 9 个时钟脉冲，释放被从机拉低的 SDA */
	for (int i = 0; i < 9; i++) {
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
		Delay_ms(2);
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		Delay_ms(2);
	}

	/* 生成 STOP 条件 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	Delay_ms(2);
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	Delay_ms(2);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
	Delay_ms(10);
}

/* ---- I2C1 硬件驱动（PB6=SCL, PB7=SDA, 100kHz） ---- */

static void I2C1_InitPins(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

static uint8_t I2C1_WriteReg(uint8_t reg, uint8_t data)
{
	uint32_t timeout;

	I2C_GenerateSTART(I2C1, ENABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	I2C_Send7bitAddress(I2C1, ICM20948_WRITE, I2C_Direction_Transmitter);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	I2C_SendData(I2C1, reg);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	I2C_SendData(I2C1, data);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	I2C_GenerateSTOP(I2C1, ENABLE);
	return 1;
}

static uint8_t I2C1_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint32_t timeout;

	/* 写寄存器地址 */
	I2C_GenerateSTART(I2C1, ENABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	I2C_Send7bitAddress(I2C1, ICM20948_WRITE, I2C_Direction_Transmitter);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	I2C_SendData(I2C1, reg);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	/* 重复起始 + 读取 */
	I2C_GenerateSTART(I2C1, ENABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	I2C_Send7bitAddress(I2C1, ICM20948_READ, I2C_Direction_Receiver);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if (--timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0; }
	}

	for (uint8_t i = 0; i < len; i++) {
		if (i == len - 1) {
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}

		timeout = 10000;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
			if (--timeout == 0) { I2C_AcknowledgeConfig(I2C1, ENABLE); return 0; }
		}
		data[i] = I2C_ReceiveData(I2C1);
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 1;
}

/* ---- 公开接口 ---- */

uint8_t IMU_Init(void)
{
	/* 1. 总线恢复（解除从机锁定） */
	I2C1_BusRecovery();
	Delay_ms(20);

	/* 2. 初始化 I2C1 硬件 */
	I2C1_InitPins();
	Delay_ms(50);

	/* 3. 唤醒 ICM20948 */
	if (!I2C1_WriteReg(REG_PWR_MGMT_1, 0x00))
		return 0;
	Delay_ms(100);

	/* 4. 验证 WHO_AM_I */
	uint8_t whoami = 0;
	if (!I2C1_ReadReg(REG_WHO_AM_I, &whoami, 1))
		return 0;
	if (whoami != 0xEA)
		return 0;

	/* 5. 陀螺仪 ±500dps (FS_SEL=01, 匹配 GYRO_SCALE=65.5) */
	if (!I2C1_WriteReg(REG_GYRO_CONFIG_1, 0x09))
		return 0;
	Delay_ms(30);

	/* 6. 校准零偏（500ms 采样） */
	float sum = 0.0f;
	for (int i = 0; i < 100; i++) {
		float w;
		if (IMU_ReadGyroZ(&w))
			sum += w;
		Delay_ms(5);
	}
	gyro_offset = sum / 100.0f;

	return 1;
}

uint8_t IMU_ReadGyroZ(float *dps)
{
	uint8_t buf[2];
	if (!I2C1_ReadReg(REG_GYRO_ZOUT_H, buf, 2))
		return 0;

	int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
	*dps = (float)raw / GYRO_SCALE - gyro_offset;
	return 1;
}

float IMU_GetGyroOffset(void)
{
	return gyro_offset;
}
