#include "IMU.h"
#include "Delay.h"

/* ============================================================================
 * IMU.c — ICM20948 九轴惯性测量单元驱动（I2C1: PB6=SCL, PB7=SDA, 0x68）
 *   ⚠ 死代码：本文件未被任何代码调用——主程序已改用 JY61P 模块
 *     （USART2 串口陀螺仪，见 JY61P.c），此处保留作参考/回退方案
 *   读取内容：仅 Z 轴角速度（循迹陀螺仪阻尼用）
 *   已知问题：GYRO_CONFIG_1 配置为 ±2000dps（0x0E），但 GYRO_SCALE=65.5
 *     对应 ±500dps 灵敏度，读数比例与实际量程不匹配（gulaoshi 原样）
 *   容错：I2C 每步超时防死锁；连续 2 次失败触发 bit-bang 总线恢复
 * ========================================================================== */

#define ICM20948_ADDR       0x68
#define ICM20948_WRITE      (ICM20948_ADDR << 1)
#define ICM20948_READ       ((ICM20948_ADDR << 1) | 0x01)

/* 寄存器 */
#define REG_WHO_AM_I        0x00
#define REG_PWR_MGMT_1      0x06
#define REG_BANK_SEL        0x7F
#define REG_PWR_MGMT_2      0x07
#define REG_LP_CONFIG       0x05
#define REG_GYRO_CONFIG_1   0x01	/* USER BANK 2 */
#define REG_GYRO_SMPLRT_DIV 0x00	/* USER BANK 2 */
#define REG_GYRO_ZOUT_H     0x37
#define REG_GYRO_ZOUT_L     0x38

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
	I2C_InitStructure.I2C_ClockSpeed = 100000;	/* SPL 轮询 400kHz 时序边缘致间歇失败，降 100k（gulaoshi HAL 才用 400k） */
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

static void I2C1_Recover(void);	/* 前置声明（WriteReg 先于定义使用） */

/* 连续失败计数：≥2 次触发总线恢复（避免偶发失败触发 62ms 阻塞） */
static uint8_t s_i2c_fail = 0;

static uint8_t I2C1_WriteReg(uint8_t reg, uint8_t data)
{
	uint32_t timeout;

	I2C_GenerateSTART(I2C1, ENABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	I2C_Send7bitAddress(I2C1, ICM20948_WRITE, I2C_Direction_Transmitter);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	I2C_SendData(I2C1, reg);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	I2C_SendData(I2C1, data);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	I2C_GenerateSTOP(I2C1, ENABLE);
	s_i2c_fail = 0;
	return 1;
}

/* 总线恢复：停外设 → bit-bang 解除从机锁定 → 重新初始化 */
static void I2C1_Recover(void)
{
	I2C_Cmd(I2C1, DISABLE);
	I2C1_BusRecovery();
	I2C1_InitPins();
}

static uint8_t I2C1_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint32_t timeout;

	/* 写寄存器地址 */
	I2C_GenerateSTART(I2C1, ENABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	I2C_Send7bitAddress(I2C1, ICM20948_WRITE, I2C_Direction_Transmitter);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	I2C_SendData(I2C1, reg);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	/* 重复起始 + 读取 */
	I2C_GenerateSTART(I2C1, ENABLE);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	I2C_Send7bitAddress(I2C1, ICM20948_READ, I2C_Direction_Receiver);
	timeout = 10000;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if (--timeout == 0) {
			I2C_GenerateSTOP(I2C1, ENABLE);
			I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
			if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
			return 0;
		}
	}

	for (uint8_t i = 0; i < len; i++) {
		if (i == len - 1) {
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}

		timeout = 10000;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
			if (--timeout == 0) {
				I2C_AcknowledgeConfig(I2C1, ENABLE);
				I2C_ClearFlag(I2C1, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
				if (++s_i2c_fail >= 2) { s_i2c_fail = 0; I2C1_Recover(); }
				return 0;
			}
		}
		data[i] = I2C_ReceiveData(I2C1);
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	s_i2c_fail = 0;
	return 1;
}

/* ---- 公开接口 ---- */

uint8_t IMU_Init(void)
{
	/* 1. 总线恢复（解除从机锁定，gulaoshi 上电执行） */
	I2C1_BusRecovery();
	Delay_ms(20);

	/* 2. 初始化 I2C1 硬件 */
	I2C1_InitPins();
	Delay_ms(50);

	/* 3. 复位 ICM20948 */
	if (!I2C1_WriteReg(REG_PWR_MGMT_1, 0x80))
		return 0;
	Delay_ms(100);

	/* 4. 选时钟源 */
	if (!I2C1_WriteReg(REG_PWR_MGMT_1, 0x01))
		return 0;
	Delay_ms(20);

	/* 5. 使能陀螺仪（gulaoshi 同序列） */
	I2C1_WriteReg(REG_PWR_MGMT_2, 0x00);
	I2C1_WriteReg(REG_LP_CONFIG, 0x00);

	/* 6. 验证 WHO_AM_I */
	uint8_t whoami = 0;
	if (!I2C1_ReadReg(REG_WHO_AM_I, &whoami, 1))
		return 0;
	if (whoami != 0xEA)
		return 0;

	/* 7. 陀螺仪 ±2000dps (FS_SEL=11, gulaoshi 原样含 bug：65.5 灵敏度不匹配) */
	I2C1_WriteReg(REG_BANK_SEL, 0x20);	/* 切到 BANK 2 */
	if (!I2C1_WriteReg(REG_GYRO_CONFIG_1, 0x0E))
		return 0;
	I2C1_WriteReg(REG_GYRO_SMPLRT_DIV, 0x00);
	I2C1_WriteReg(REG_BANK_SEL, 0x00);	/* 切回 BANK 0 */
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
	/* 分两次 1 字节读：避开 2 字节读取的 ACK/STOP 时序坑（1 字节路径已验证可靠） */
	uint8_t hi, lo;
	if (!I2C1_ReadReg(REG_GYRO_ZOUT_H, &hi, 1))
		return 0;
	if (!I2C1_ReadReg(REG_GYRO_ZOUT_L, &lo, 1))
		return 0;

	int16_t raw = ((int16_t)hi << 8) | lo;
	*dps = (float)raw / GYRO_SCALE - gyro_offset;
	return 1;
}

float IMU_GetGyroOffset(void)
{
	return gyro_offset;
}
