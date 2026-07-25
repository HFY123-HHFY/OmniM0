#include "ICM42688.h"
#include "API_SPI.h"
#include "BusRate.h"
#include "Delay.h"
#include <math.h>

/*
 * ICM-42688-P 6 轴 IMU 驱动 — ISR 驱动 + 缓存模型
 *
 *   ISR → ReadSensor()          一次 SPI burst，float 转换，偏航积分
 *   主循环 → GetXxx()           零 SPI，只读缓存
 *
 * 速度优化：
 *   - Bus 选择只做一次（Init 时切换），后续 ReadSensor 不再 SelectBus
 *   - 12 字节 burst 读（0x1F→0x2A），一次 CS↓↑
 *   - 5MHz DelayOff 实际 SCK ≈ 10~15MHz
 *   - 单次 ReadSensor ≈ 180μs（含 SPI 12μs + float 转换 160μs）
 */

/* ── 缓存的传感器数据 ── */
static float    s_ax, s_ay, s_az;      /* 加速度 (m/s²)           */
static float    s_gx, s_gy, s_gz;      /* 角速度 (°/s)            */
static float    s_roll, s_pitch;       /* 横滚 / 俯仰 (°)        */
static float    s_yaw;                 /* 偏航积分 (°)           */
static uint32_t s_lastTick;            /* 上次 ReadSensor 的 tick */
static uint8_t  s_firstRead;           /* 首次读取标志            */

/* ── 量程系数 + 零偏 ── */
static float s_accelScale = 1.0f;      /* LSB → m/s² */
static float s_gyroScale  = 1.0f;      /* LSB → °/s  */
static float s_gyroBiasZ  = 0.0f;      /* Z 轴零偏 (°/s)，Init 时校准 */
static uint8_t s_inited = 0U;

/* ── 系统 tick 外部引用（Control_Task.c）── */
extern volatile uint32_t g_sys_tick_ms;

/* ══════════════════════════════════════════════════════════
 * SPI 底层（内部）
 * ══════════════════════════════════════════════════════════ */

/*
 * 写单个寄存器 — 仅供 Init / SetRange 使用，不参与热路径
 */
static void ICM42688_WriteReg(uint8_t reg, uint8_t data)
{
	API_SPI_SelectBus(ICM42688_SPI_BUS);
	API_SPI_Start();
	API_SPI_SwapByte(reg & 0x7FU);
	API_SPI_SwapByte(data);
	API_SPI_Stop();
}

/*
 * 读单个寄存器
 */
static uint8_t ICM42688_ReadReg(uint8_t reg)
{
	uint8_t val;
	API_SPI_SelectBus(ICM42688_SPI_BUS);
	API_SPI_Start();
	API_SPI_SwapByte(reg | 0x80U);
	val = API_SPI_SwapByte(0xFFU);
	API_SPI_Stop();
	return val;
}

/*
 * 极速 burst 读 — 热路径，不做 bus 选择（假设已选 SPI2）
 *
 * 从 reg 开始连续读 len 字节。CS 在一次事务内保持低。
 */
static void ICM42688_BurstReadFast(uint8_t reg, uint8_t *buf, uint8_t len)
{
	API_SPI_Start();
	API_SPI_SwapByte(reg | 0x80U);
	while (len--)
	{
		*buf++ = API_SPI_SwapByte(0xFFU);
	}
	API_SPI_Stop();
}

/* ══════════════════════════════════════════════════════════
 * 初始化
 * ══════════════════════════════════════════════════════════ */

uint8_t ICM42688_Init(void)
{
	uint8_t whoami, retry;

	/* ── 切到 SPI2，设最高速 ── */
	API_SPI_SelectBus(ICM42688_SPI_BUS);
	API_SPI_SetSpeed(ICM42688_SPI_SPEED);
	API_SPI_DelayOff();

	/* ── WHO_AM_I（重试最多 50 次，每次等 10ms，合计 500ms）── */
	for (retry = 0U; retry < 50U; ++retry)
	{
		whoami = ICM42688_ReadReg(ICM42688_WHO_AM_I);
		if (whoami == ICM42688_WHO_AM_I_VAL) { break; }
		Delay_ms(10U);
	}
	if (whoami != ICM42688_WHO_AM_I_VAL)
	{
		s_inited = 0U;   /* 标记未初始化，ReadSensor/GetXxx 将跳过 */
		return 0U;       /* 返回失败，调用方可做 LED 告警 / fallback */
	}

	/* ── 软复位 ── */
	ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x00U);
	Delay_ms(10U);

	/* ── 量程 / ODR：±16g, ±2000dps, 双 1kHz ── */
	ICM42688_WriteReg(ICM42688_ACCEL_CONFIG0,
	                  (uint8_t)((ICM42688_ACCEL_16G << 5) | (ICM42688_ODR_1KHZ + 1U)));
	ICM42688_WriteReg(ICM42688_GYRO_CONFIG0,
	                  (uint8_t)((ICM42688_GYRO_2000DPS << 5) | (ICM42688_ODR_1KHZ + 1U)));

	/* 量程系数 */
	s_accelScale = 16.0f / 32768.0f * 9.80665f;     /* ≈ 0.004789 m/s²/LSB */
	s_gyroScale  = 2000.0f / 32768.0f;               /* ≈ 0.06104 °/s/LSB  */

	/* ── 低噪声使能 ── */
	ICM42688_WriteReg(ICM42688_GYRO_CONFIG1, 0x06U);

	/* ── 上电：GYRO + ACCEL 低噪声模式 ── */
	ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x0FU);
	Delay_ms(50U);   /* 低噪声模式启动需 ~30ms，取 50ms 充裕 */

	/*
	 * ── 陀螺仪 Z 轴零偏校准 ──
	 * 静止状态下采 50 次，每次间隔 1ms（匹配 1kHz ODR），取平均。
	 * 50 × 1ms = 50ms，覆盖 50 个独立 gyro 样本。
	 */
	{
		uint8_t  calBuf[12];
		int16_t  rawGz;
		float    biasSum = 0.0f;
		uint16_t i;

		for (i = 0U; i < 50U; ++i)
		{
			ICM42688_BurstReadFast(ICM42688_ACCEL_DATA_X1, calBuf, 12U);
			rawGz = (int16_t)(((uint16_t)calBuf[10] << 8) | calBuf[11]);
			biasSum += (float)rawGz * s_gyroScale;
			Delay_ms(1U);   /* 等下一个 ODR 周期，确保每次都是独立样本 */
		}
		s_gyroBiasZ = biasSum / 50.0f;
	}

	/* ── 偏航积分状态初始化 ── */
	s_yaw       = 0.0f;
	s_firstRead = 1U;
	s_inited    = 1U;

	return 1U;   /* 成功 */
}

/* ══════════════════════════════════════════════════════════
 * ISR 传感器读取（核心热路径）
 * ══════════════════════════════════════════════════════════ */

void ICM42688_ReadSensor(void)
{
	uint8_t  buf[12];
	int16_t  rawAx, rawAy, rawAz;
	int16_t  rawGx, rawGy, rawGz;
	float    ax, ay, az, gx, gy, gz;
	uint32_t now;
	float    dt;

	if (s_inited == 0U) { return; }

	/*
	 * ── 阶段 1：SPI burst 读 12 字节 ──
	 * 起始地址 0x1F (ACCEL_DATA_X1)，连续读到 0x2A (GYRO_DATA_Z0)
	 * 假设 SPI2 已由 Init 选中且未被他设备切换（OLED=SPI1, 不冲突）
	 */
	ICM42688_BurstReadFast(ICM42688_ACCEL_DATA_X1, buf, 12U);

	/* Big-endian → int16_t */
	rawAx = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
	rawAy = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
	rawAz = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
	rawGx = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
	rawGy = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
	rawGz = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);

	/*
	 * ── 阶段 2：浮点转换 ──
	 */
	ax = (float)rawAx * s_accelScale;
	ay = (float)rawAy * s_accelScale;
	az = (float)rawAz * s_accelScale;
	gx = (float)rawGx * s_gyroScale;
	gy = (float)rawGy * s_gyroScale;
	gz = (float)rawGz * s_gyroScale;

	/* ── 更新全局缓存（gyro_z 已减零偏）── */
	s_ax = ax;  s_ay = ay;  s_az = az;
	s_gx = gx;  s_gy = gy;
	s_gz = gz - s_gyroBiasZ;     /* 零偏校正后的角速度 */
	if (s_gz > -ICM42688_GYRO_DEADBAND && s_gz < ICM42688_GYRO_DEADBAND)
	{
		s_gz = 0.0f;             /* 死区抑制：残余零偏不积分 */
	}

	/* roll / pitch：atan2 反算 */
	s_roll  = atan2f(ay, az) * 57.29578f;
	{
		float norm = sqrtf(ay * ay + az * az);
		s_pitch = atan2f(-ax, norm) * 57.29578f;
	}

	/*
	 * ── 阶段 3：偏航积分（零偏已校正）──
	 */
	now = g_sys_tick_ms;
	if (s_firstRead)
	{
		s_yaw       = 0.0f;
		s_lastTick  = now;
		s_firstRead = 0U;
	}
	else
	{
		dt = (float)(now - s_lastTick) * 0.001f;
		if (dt > 0.0f && dt < 0.1f)
		{
			s_yaw += s_gz * dt;     /* 使用校正后的 gz */
		}
		s_lastTick = now;
	}

	/* ── Yaw 限幅到 ±180° ── */
	while (s_yaw >  180.0f) { s_yaw -= 360.0f; }
	while (s_yaw < -180.0f) { s_yaw += 360.0f; }
}

/* ══════════════════════════════════════════════════════════
 * 数据获取（只读缓存，零 SPI）
 * ══════════════════════════════════════════════════════════ */

void ICM42688_GetAttitude(float *roll, float *pitch, float *yaw)
{
	if (roll  != 0) { *roll  = s_roll;  }
	if (pitch != 0) { *pitch = s_pitch; }
	if (yaw   != 0) { *yaw   = s_yaw;   }
}

void ICM42688_GetGyroscope(float *gx, float *gy, float *gz)
{
	if (gx != 0) { *gx = s_gx; }
	if (gy != 0) { *gy = s_gy; }
	if (gz != 0) { *gz = s_gz; }
}

void ICM42688_GetAccelerometer(float *ax, float *ay, float *az)
{
	if (ax != 0) { *ax = s_ax; }
	if (ay != 0) { *ay = s_ay; }
	if (az != 0) { *az = s_az; }
}
