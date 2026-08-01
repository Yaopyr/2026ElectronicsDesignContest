#include "stm32f10x.h"
#include "Delay.h"
#include "Tracking.h"
#include "Motor.h"
#include "OLED.h"
#include "UART.h"
#include "Stepper.h"
#include "Encoder.h"
#include "JY61P.h"

#include "pid.h"

/* 球平衡 PID */
#define PID_KP		1.2
#define PID_KI		0.25//0.2
#define PID_KD		0.35

/* 循迹 PID（来自 gulaoshi track_ctrl.c） */
#define TRACK_KP      40.0f//30.0f//15.0f//25.0f	/* 小误差修正温和（直道）；弯道误差大走饱和区不受影响 */
#define TRACK_KI      0.3f
#define GYRO_D_GAIN   5.0f//10.0f//5.0f//15.0f//20.0f//15.0f//10.0f	/* 弯道中低速段阻尼翻倍；饱和区由DIFF限幅主导 */
#define DIFF_MAX      120.0f//150.0f//200.0f//150.0f//250.0f
#define DIFF_MIN     -120.0f//-150.0f//-200.0f//-150.0f//-250.0f
#define I_MAX         80.0f
#define I_SEP         15.0f
#define TRACK_OUT_MAX 1000.0f	/* PID 内部限幅（gulaoshi 双级限幅） */

/* MODE_06 平衡球目标调节（单位 cm；实测：+5cm→340, +7cm→490, +10cm→710 → setpoint = 74×cm − 30） */
#define TARGET_SP_SLOPE   74.0f			/* 单位/cm */
#define TARGET_SP_OFFSET -30.0f			/* 零点偏移 */
#define TARGET_K          0.02f			/* 左轮编码器增量 → cm（需实测标定） */
#define TARGET_MAX        12.0f			/* ±12.0 cm 双向限幅 */

/* MODE_06 特调球 PID（target ≤ −4.0cm 时，按开始后生效） */
#define PID06_KP  1.0f
#define PID06_KI  0.25f
#define PID06_KD  0.20f

volatile uint32_t g_elapsed_ms;

/* ========== 循迹 PID（与 gulaoshi pid.c 算法一致） ========== */
typedef struct {
	float Kp, Ki, Kd;
	float error, last_error, integral;
	float output;
	float out_max, out_min;
	float i_max, i_sep;
} TrackPID;

static void TrackPID_Reset(TrackPID *pid)
{
	pid->error = 0.0f;
	pid->last_error = 0.0f;
	pid->integral = 0.0f;
	pid->output = 0.0f;
}

static void TrackPID_Init(TrackPID *pid, float kp, float ki, float kd,
                           float out_min, float out_max, float i_max, float i_sep)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->out_min = out_min;
	pid->out_max = out_max;
	pid->i_max = i_max;
	pid->i_sep = i_sep;
	TrackPID_Reset(pid);
}

static float TrackPID_Update(TrackPID *pid, float error)
{
	pid->error = error;

	float abs_err = (pid->error > 0.0f) ? pid->error : -pid->error;
	if (pid->i_sep <= 0.0f || abs_err <= pid->i_sep) {
		pid->integral += pid->error;
		if (pid->integral > pid->i_max)
			pid->integral = pid->i_max;
		else if (pid->integral < -pid->i_max)
			pid->integral = -pid->i_max;
	}

	float derivative = pid->error - pid->last_error;
	float output = pid->Kp * pid->error
	             + pid->Ki * pid->integral
	             + pid->Kd * derivative;

	if (output > pid->out_max) {
		output = pid->out_max;
		if (pid->Ki != 0.0f && pid->error > 0.0f)
			pid->integral -= pid->error;
	} else if (output < pid->out_min) {
		output = pid->out_min;
		if (pid->Ki != 0.0f && pid->error < 0.0f)
			pid->integral -= pid->error;
	}

	pid->last_error = pid->error;
	pid->output = output;
	return output;
}

typedef enum {
	STATE_WAIT,
	STATE_ACCEL,
	STATE_RUN,
	STATE_DECEL,
	STATE_BACK,		/* MODE_02 停止后全速后退 */
	STATE_DONE
} State_TypeDef;

typedef enum {
	MODE_02,	// 循迹 + 平衡球
	MODE_03,	// 定点测试：+340 → -340
	MODE_04,	// 循迹4秒停止（缓缓启停）
	MODE_05,	// 循迹（降速20%+缓启缓停+触发后3s再缓停）
	MODE_06		// 循迹同05 + 编码器调平衡球目标（±12mm）
} Mode_TypeDef;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	OLED_Init();
	Tracking_Init();
	Motor_Init();
	UART_Init();
	Stepper_Init();
	Encoder_Init();

	/* JY61P 串口初始化，失败则无陀螺仪阻尼 */
	uint8_t imu_ok = JY61P_Init();
	if (!imu_ok)
		OLED_ShowString(1, 1, "IMU OFF");

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

	/* 球平衡 PID */
	double pidInput = 0, pidOutput = 0, pidSetpoint = 0;
	PID_TypeDef pid;
	PID(&pid, &pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD,
	    _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&pid, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&pid, 20);
	PID_SetOutputLimits(&pid, -STEP_RANGE, STEP_RANGE);

	/* 循迹 PID */
	TrackPID track_pid;
	TrackPID_Init(&track_pid, TRACK_KP, TRACK_KI, 0.0f,
	              -TRACK_OUT_MAX, TRACK_OUT_MAX, I_MAX, I_SEP);

	/* PB0 模式键 + PB1 启动键（IPU） */
	GPIO_InitTypeDef GPIO_InitStructure2;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure2);

	/* 状态机 */
	State_TypeDef state = STATE_WAIT;
	Mode_TypeDef mode = MODE_02;

	while (1)
	{
		static uint8_t pb0_prev = 1;
		static int16_t ramp_speed = 0;
		static uint32_t decel_time = 0;
		static uint32_t back_start_ms = 0;	/* MODE_02 停止后全速后退起点 */
		static uint32_t extend_start = 0;	/* MODE_05 触发后延长循迹起点 */
		static uint8_t  extending    = 0;	/* MODE_05 延长循迹中 */
		static float ball_target = 0.0f;	/* MODE_06 平衡球目标(cm)，默认 00.0 */
		static uint8_t pid06_low = 0xFF;	/* MODE_06 特调选择：0=默认 1=特调 0xFF=未在06 */

		/* 两阶段终点检测 */
		static uint8_t finish_armed = 0;
		static uint8_t finish_cnt   = 0;
		static uint8_t leave_cnt    = 0;

		/* 丢线检测 */
		static uint8_t lost_cnt = 0;

		uint8_t m056 = (mode == MODE_05 || mode == MODE_06);	/* MODE_05/06 共用特调参数 */
		uint16_t bs = m056 ? 240 : 300;	/* MODE_05/06 全程降速20% */
		uint16_t step = (mode == MODE_04) ? 4 : (m056) ? 3 : 80;	/* MODE_04/05/06 缓启缓停约1.5s；其余 gulaoshi 4000/s */
		float gyro_gain = m056 ? 1.8f : GYRO_D_GAIN;	/* MODE_05/06 陀螺仪阻尼 1.8，其余 5 */

		/* 编码器速度更新 */
		Encoder_Update();

		/* 陀螺仪读取（带容错） */
		float omega_z = 0.0f;
		static float last_valid_omega = 0.0f;
		static uint8_t g_imu_ok = 0;
		if (JY61P_GetWz(&omega_z)) {
			last_valid_omega = omega_z;
			g_imu_ok = 1;
		} else {
			omega_z = last_valid_omega;
			g_imu_ok = 0;
		}

		/* MODE_06：target ≤ −4.0cm 实时切特调球 PID（含调节阶段与开始后） */
		if (mode == MODE_06)
		{
			uint8_t low = (ball_target <= -4.0f);
			if (low != pid06_low)
			{
				PID_SetTunings(&pid, low ? PID06_KP : PID_KP,
				                      low ? PID06_KI : PID_KI,
				                      low ? PID06_KD : PID_KD);
				pid06_low = low;
			}
		}
		else if (pid06_low != 0xFF)
		{
			PID_SetTunings(&pid, PID_KP, PID_KI, PID_KD);	/* 切出06恢复默认 */
			pid06_low = 0xFF;
		}

		switch (state)
		{
		case STATE_ACCEL:
			if (mode == MODE_03) { state = STATE_RUN; break; }
			{
				if (ramp_speed + step < bs)
					ramp_speed += step;
				else
					ramp_speed = bs;

				float error = Tracking_GetError();
				float diff = -TrackPID_Update(&track_pid, error);	/* 差速取反：修正转向方向（位序/电机接反） */
				diff -= gyro_gain * omega_z;	/* 阻尼：JY61P 顺时针为正，实测须取反才构成抑制 */
				if (diff > DIFF_MAX) diff = DIFF_MAX;
				if (diff < DIFF_MIN) diff = DIFF_MIN;

				int16_t left  = (int16_t)ramp_speed + (int16_t)diff;
				int16_t right = (int16_t)ramp_speed - (int16_t)diff;

				if (left  > 999)  left  = 999;
				if (left  < -999) left  = -999;
				if (right > 999)  right = 999;
				if (right < -999) right = -999;

				Motor_SetLeft( left);
				Motor_SetRight(right);

				/* 丢线检测 */
				if (g_lt_raw == 0xFF) {
					if (++lost_cnt >= 5) {
						Motor_Stop();
						state = STATE_DONE;
						break;
					}
				} else {
					lost_cnt = 0;
				}

				/* 平衡球 */
				pidInput = (double)UART_GetBallPosition();
				PID_Compute(&pid);
				Stepper_SetTarget((int32_t)pidOutput);
				OLED_ShowString(1, 1, "T:");
				OLED_ShowNum(1, 3, g_elapsed_ms / 1000, 4);
				OLED_ShowString(1, 7, "s");

				/* 陀螺仪角速度（dps） */
				OLED_ShowString(1, 9, "W:");
				OLED_ShowSignedNum(1, 11, (int32_t)omega_z, 4);
				OLED_ShowString(1, 15, g_imu_ok ? "OK" : "ER");

				/* 左右轮速（编码器 20ms 增量） */
				OLED_ShowString(2, 9, "L:");
				OLED_ShowSignedNum(2, 11, (int32_t)Encoder_GetSpeedLeft(), 4);
				OLED_ShowString(3, 9, "R:");
				OLED_ShowSignedNum(3, 11, (int32_t)Encoder_GetSpeedRight(), 4);

				/* 循迹传感器值（黑线=0）+ 读状态 */
				OLED_ShowString(4, 1, "LT:");
				OLED_ShowBinNum(4, 4, g_lt_raw, 8);
				OLED_ShowString(4, 13, g_lt_ok ? "OK" : "ER");

				if (ramp_speed >= bs)
					state = STATE_RUN;
			}
			break;

		case STATE_WAIT:
			if (mode == MODE_06)
			{
				/* 左轮编码器调节目标：正转增加/反转减少，实时作用于平衡球 */
				ball_target += (float)Encoder_GetSpeedLeft() * TARGET_K;
				if (ball_target >  TARGET_MAX) ball_target =  TARGET_MAX;
				if (ball_target < -TARGET_MAX) ball_target = -TARGET_MAX;
				pidSetpoint = (double)(ball_target * TARGET_SP_SLOPE + TARGET_SP_OFFSET);
			}
			else
			{
				pidSetpoint = 0;	/* 切回其他模式时球回中 */
			}

			/* 平衡球 */
			pidInput = (double)UART_GetBallPosition();
			PID_Compute(&pid);
			Stepper_SetTarget((int32_t)pidOutput);

			OLED_ShowString(1, 1, "M:");
			OLED_ShowNum(1, 3, (mode == MODE_02) ? 2 : (mode == MODE_03) ? 3 : (mode == MODE_04) ? 4 : (mode == MODE_05) ? 5 : 6, 2);
			if (mode == MODE_06)
			{
				/* Target: xx.x cm（±12.0 双向） */
				int16_t t10 = (int16_t)(ball_target * 10.0f);
				OLED_ShowString(2, 1, "Target:");
				if (t10 < 0) { OLED_ShowString(2, 8, "-"); OLED_ShowNum(2, 9, -t10 / 10, 2); }
				else         { OLED_ShowString(2, 8, " "); OLED_ShowNum(2, 9,  t10 / 10, 2); }
				OLED_ShowString(2, 11, ".");
				OLED_ShowNum(2, 12, (t10 < 0 ? -t10 : t10) % 10, 1);
				OLED_ShowString(2, 13, "cm");
				OLED_ShowString(3, 1, "P:");
				OLED_ShowSignedNum(3, 3, (int32_t)pidInput, 4);
			}
			else
			{
				OLED_ShowString(2, 1, "P:");
				OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
				OLED_ShowString(3, 1, "T:");
				OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);
			}

			{
				uint8_t pb0_now = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
				if (pb0_prev == 1 && pb0_now == 0)
					mode = (mode == MODE_02) ? MODE_03 : (mode == MODE_03) ? MODE_04 : (mode == MODE_04) ? MODE_05 : (mode == MODE_05) ? MODE_06 : MODE_02;
				pb0_prev = pb0_now;
			}

			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
			{
				pidSetpoint = (mode == MODE_06) ? (double)(ball_target * TARGET_SP_SLOPE + TARGET_SP_OFFSET) : 0;
				PID_SetMode(&pid, _PID_MODE_AUTOMATIC);
				TrackPID_Reset(&track_pid);
				OLED_Clear();
				g_elapsed_ms = 0;
				ramp_speed = 0;
				decel_time = 0;
				back_start_ms = 0;
				extend_start = 0;
				extending    = 0;
				finish_armed = 0;
				finish_cnt = 0;
				leave_cnt = 0;
				lost_cnt = 0;
				state = STATE_ACCEL;
			}
			break;

		case STATE_RUN:
			if (mode == MODE_02)
			{
				/* MODE_02 终点检测：3+ 传感器见黑线且启动超 10s → 直接停车 */
				{
					uint8_t black_cnt = Tracking_GetBlackCount();
					if (black_cnt >= 4 && g_elapsed_ms > 17000)
					{
						decel_time = g_elapsed_ms;	/* 触发即冻结计时 */
						Motor_Stop();
						OLED_Clear();
						OLED_ShowString(1, 1, "Time:");
						OLED_ShowNum(2, 1, decel_time / 1000, 5);
						OLED_ShowString(2, 7, "s");
						back_start_ms = g_elapsed_ms;	/* 记录后退起点 */
						state = STATE_BACK;
						break;
					}
				}

				/* 丢线检测 */
				if (g_lt_raw == 0xFF) {
					if (++lost_cnt >= 5) {
						Motor_Stop();
						state = STATE_DONE;
						break;
					}
				} else {
					lost_cnt = 0;
				}

				OLED_ShowString(1, 1, "T:");
				OLED_ShowNum(1, 3, g_elapsed_ms / 1000, 4);
				OLED_ShowString(1, 7, "s");

				/* 陀螺仪角速度（dps） */
				OLED_ShowString(1, 9, "W:");
				OLED_ShowSignedNum(1, 11, (int32_t)omega_z, 4);
				OLED_ShowString(1, 15, g_imu_ok ? "OK" : "ER");

				/* 左右轮速（编码器 20ms 增量） */
				OLED_ShowString(2, 9, "L:");
				OLED_ShowSignedNum(2, 11, (int32_t)Encoder_GetSpeedLeft(), 4);
				OLED_ShowString(3, 9, "R:");
				OLED_ShowSignedNum(3, 11, (int32_t)Encoder_GetSpeedRight(), 4);

				/* 循迹传感器值（黑线=0）+ 读状态 */
				OLED_ShowString(4, 1, "LT:");
				OLED_ShowBinNum(4, 4, g_lt_raw, 8);
				OLED_ShowString(4, 13, g_lt_ok ? "OK" : "ER");

				float error = Tracking_GetError();
				float diff = -TrackPID_Update(&track_pid, error);	/* 差速取反：修正转向方向（位序/电机接反） */
				diff -= gyro_gain * omega_z;	/* 阻尼：JY61P 顺时针为正，实测须取反才构成抑制 */
				if (diff > DIFF_MAX) diff = DIFF_MAX;
				if (diff < DIFF_MIN) diff = DIFF_MIN;

				int16_t left  = (int16_t)bs + (int16_t)diff;
				int16_t right = (int16_t)bs - (int16_t)diff;

				if (left  > 999)  left  = 999;
				if (left  < -999) left  = -999;
				if (right > 999)  right = 999;
				if (right < -999) right = -999;

				Motor_SetLeft( left);
				Motor_SetRight(right);

				pidInput = (double)UART_GetBallPosition();
				PID_Compute(&pid);
				Stepper_SetTarget((int32_t)pidOutput);

				OLED_ShowString(2, 1, "P:");
				OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
				OLED_ShowString(3, 1, "T:");
				OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);
			}
			else if (mode == MODE_04)
			{
				if (g_elapsed_ms >= 5000)
				{
					ramp_speed = bs;
					decel_time = g_elapsed_ms;
					state = STATE_DECEL;
					break;
				}

				if (g_lt_raw == 0xFF) {
					if (++lost_cnt >= 5) {
						Motor_Stop();
						state = STATE_DONE;
						break;
					}
				} else {
					lost_cnt = 0;
				}

				OLED_ShowString(1, 1, "T:");
				OLED_ShowNum(1, 3, g_elapsed_ms / 1000, 4);
				OLED_ShowString(1, 7, "s");

				/* 陀螺仪角速度（dps） */
				OLED_ShowString(1, 9, "W:");
				OLED_ShowSignedNum(1, 11, (int32_t)omega_z, 4);
				OLED_ShowString(1, 15, g_imu_ok ? "OK" : "ER");

				/* 左右轮速（编码器 20ms 增量） */
				OLED_ShowString(2, 9, "L:");
				OLED_ShowSignedNum(2, 11, (int32_t)Encoder_GetSpeedLeft(), 4);
				OLED_ShowString(3, 9, "R:");
				OLED_ShowSignedNum(3, 11, (int32_t)Encoder_GetSpeedRight(), 4);

				/* 循迹传感器值（黑线=0）+ 读状态 */
				OLED_ShowString(4, 1, "LT:");
				OLED_ShowBinNum(4, 4, g_lt_raw, 8);
				OLED_ShowString(4, 13, g_lt_ok ? "OK" : "ER");

				float error = Tracking_GetError();
				float diff = -TrackPID_Update(&track_pid, error);	/* 差速取反：修正转向方向（位序/电机接反） */
				diff -= gyro_gain * omega_z;	/* 阻尼：JY61P 顺时针为正，实测须取反才构成抑制 */
				if (diff > DIFF_MAX) diff = DIFF_MAX;
				if (diff < DIFF_MIN) diff = DIFF_MIN;

				int16_t left  = (int16_t)bs + (int16_t)diff;
				int16_t right = (int16_t)bs - (int16_t)diff;

				if (left  > 999)  left  = 999;
				if (left  < -999) left  = -999;
				if (right > 999)  right = 999;
				if (right < -999) right = -999;

				Motor_SetLeft( left);
				Motor_SetRight(right);

				pidInput = (double)UART_GetBallPosition();
				PID_Compute(&pid);
				Stepper_SetTarget((int32_t)pidOutput);

				OLED_ShowString(2, 1, "P:");
				OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
				OLED_ShowString(3, 1, "T:");
				OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);
			}
			else if (m056)
			{
				/* 触发后延长循迹 3s 再进入缓停 */
				if (extending && g_elapsed_ms - extend_start >= 3000)
				{
					ramp_speed = bs;
					extending = 0;
					state = STATE_DECEL;
					break;
				}

				{
					uint8_t black_cnt = Tracking_GetBlackCount();

					if (!finish_armed) {
						if (black_cnt < 3) {
							if (++leave_cnt >= 5) {
								finish_armed = 1;
								finish_cnt = 0;
							}
						} else {
							leave_cnt = 0;
						}
					} else {
						if (black_cnt >= 3) {
							if (!extending && ++finish_cnt >= 5) {
								decel_time = g_elapsed_ms;	/* 触发即冻结计时 */
								extend_start = g_elapsed_ms;	/* 触发后继续循迹3s再缓停 */
								extending = 1;
							}
						} else {
							finish_cnt = 0;
						}
					}
				}

				if (g_lt_raw == 0xFF) {
					if (++lost_cnt >= 5) {
						Motor_Stop();
						state = STATE_DONE;
						break;
					}
				} else {
					lost_cnt = 0;
				}

				OLED_ShowString(1, 1, "T:");
				OLED_ShowNum(1, 3, g_elapsed_ms / 1000, 4);
				OLED_ShowString(1, 7, "s");

				/* 陀螺仪角速度（dps） */
				OLED_ShowString(1, 9, "W:");
				OLED_ShowSignedNum(1, 11, (int32_t)omega_z, 4);
				OLED_ShowString(1, 15, g_imu_ok ? "OK" : "ER");

				/* 左右轮速（编码器 20ms 增量） */
				OLED_ShowString(2, 9, "L:");
				OLED_ShowSignedNum(2, 11, (int32_t)Encoder_GetSpeedLeft(), 4);
				OLED_ShowString(3, 9, "R:");
				OLED_ShowSignedNum(3, 11, (int32_t)Encoder_GetSpeedRight(), 4);

				/* 循迹传感器值（黑线=0）+ 读状态 */
				OLED_ShowString(4, 1, "LT:");
				OLED_ShowBinNum(4, 4, g_lt_raw, 8);
				OLED_ShowString(4, 13, g_lt_ok ? "OK" : "ER");

				float error = Tracking_GetError();
				float diff = -TrackPID_Update(&track_pid, error);	/* 差速取反：修正转向方向（位序/电机接反） */
				diff -= gyro_gain * omega_z;	/* 阻尼：JY61P 顺时针为正，实测须取反才构成抑制 */
				if (diff > DIFF_MAX) diff = DIFF_MAX;
				if (diff < DIFF_MIN) diff = DIFF_MIN;

				int16_t left  = (int16_t)bs + (int16_t)diff;
				int16_t right = (int16_t)bs - (int16_t)diff;

				if (left  > 999)  left  = 999;
				if (left  < -999) left  = -999;
				if (right > 999)  right = 999;
				if (right < -999) right = -999;

				Motor_SetLeft( left);
				Motor_SetRight(right);

				pidInput = (double)UART_GetBallPosition();
				PID_Compute(&pid);
				Stepper_SetTarget((int32_t)pidOutput);

				OLED_ShowString(2, 1, "P:");
				OLED_ShowSignedNum(2, 3, (int32_t)pidInput, 4);
				OLED_ShowString(3, 1, "T:");
				OLED_ShowSignedNum(3, 3, (int32_t)pidOutput, 4);
			}
			else /* MODE_03 */
			{
				static int8_t  sub03 = 0;
				static uint8_t stable_cnt = 0;
				int32_t error;

				switch (sub03)
				{
				case 0:
					pidSetpoint = 340;
					sub03 = 1;
					stable_cnt = 0;
					break;

				case 1:
					error = (int32_t)(pidInput - 340);
					if (error < 0) error = -error;
					if (error <= 40)
					{
						if (++stable_cnt >= 5)
						{
							sub03 = 2;
							stable_cnt = 0;
						}
					}
					else
						stable_cnt = 0;
					break;

				case 2:
					pidSetpoint = -340;
					sub03 = 3;
					stable_cnt = 0;
					break;

				case 3:
					error = (int32_t)(pidInput + 340);
					if (error < 0) error = -error;
					if (error <= 50)
					{
						if (++stable_cnt >= 5)
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

		case STATE_DECEL:
			{
				if (ramp_speed > step)
					ramp_speed -= step;
				else
					ramp_speed = 0;

				float error = Tracking_GetError();
				float diff = -TrackPID_Update(&track_pid, error);	/* 差速取反：修正转向方向（位序/电机接反） */
				diff -= gyro_gain * omega_z;	/* 阻尼：JY61P 顺时针为正，实测须取反才构成抑制 */
				if (diff > DIFF_MAX) diff = DIFF_MAX;
				if (diff < DIFF_MIN) diff = DIFF_MIN;

				int16_t left  = (int16_t)ramp_speed + (int16_t)diff;
				int16_t right = (int16_t)ramp_speed - (int16_t)diff;

				if (left  > 999)  left  = 999;
				if (left  < -999) left  = -999;
				if (right > 999)  right = 999;
				if (right < -999) right = -999;

				Motor_SetLeft( left);
				Motor_SetRight(right);

				pidInput = (double)UART_GetBallPosition();
				PID_Compute(&pid);
				Stepper_SetTarget((int32_t)pidOutput);

				OLED_ShowString(1, 1, "T:");
				OLED_ShowNum(1, 3, g_elapsed_ms / 1000, 4);
				OLED_ShowString(1, 7, "s");

				/* 陀螺仪角速度（dps） */
				OLED_ShowString(1, 9, "W:");
				OLED_ShowSignedNum(1, 11, (int32_t)omega_z, 4);
				OLED_ShowString(1, 15, g_imu_ok ? "OK" : "ER");

				/* 左右轮速（编码器 20ms 增量） */
				OLED_ShowString(2, 9, "L:");
				OLED_ShowSignedNum(2, 11, (int32_t)Encoder_GetSpeedLeft(), 4);
				OLED_ShowString(3, 9, "R:");
				OLED_ShowSignedNum(3, 11, (int32_t)Encoder_GetSpeedRight(), 4);

				/* 循迹传感器值（黑线=0）+ 读状态 */
				OLED_ShowString(4, 1, "LT:");
				OLED_ShowBinNum(4, 4, g_lt_raw, 8);
				OLED_ShowString(4, 13, g_lt_ok ? "OK" : "ER");

				if (ramp_speed == 0)
				{
					OLED_Clear();
					OLED_ShowString(1, 1, "Time:");
					OLED_ShowNum(2, 1, decel_time / 1000, 5);
					OLED_ShowString(2, 7, "s");
					state = STATE_DONE;
				}
			}
			break;

		case STATE_BACK:
			if (g_elapsed_ms - back_start_ms >= 100)
			{
				Motor_Stop();
				state = STATE_DONE;
				break;
			}
			Motor_SetLeft(-999);	/* 全速后退 */
			Motor_SetRight(-999);

			/* 平衡球 */
			pidInput = (double)UART_GetBallPosition();
			PID_Compute(&pid);
			Stepper_SetTarget((int32_t)pidOutput);
			break;

		case STATE_DONE:
			Motor_Stop();	/* 清残留差速，防止原地打转 */
			pidInput = (double)UART_GetBallPosition();
			PID_Compute(&pid);
			Stepper_SetTarget((int32_t)pidOutput);

			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
			{
				TrackPID_Reset(&track_pid);
				OLED_Clear();
				g_elapsed_ms = 0;
				ramp_speed = 0;
				decel_time = 0;
				back_start_ms = 0;
				extend_start = 0;
				extending    = 0;
				finish_armed = 0;
				finish_cnt = 0;
				leave_cnt = 0;
				lost_cnt = 0;
				state = STATE_ACCEL;
			}
			break;
		}

		Delay_ms(20);
	}
}
