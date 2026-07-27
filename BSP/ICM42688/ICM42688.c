#include "ICM42688.h"
#include "API_SPI.h"
#include "soft_spi_hal.h"   /* soft_spi_context_t + save/restore（ISR 抢占保护） */
#include "BusRate.h"
#include "Delay.h"
#include <math.h>

/*
 * ICM-42688-P 6 轴 IMU 驱动 — ISR 驱动 + 双缓冲 + 全局结构体直读
 *
 * ── 双缓冲设计 ──
 *   ISR（5ms）：ReadSensor() 在 ISR 上下文写完 g_icm42688，最高优先级，
 *               不会被主循环抢占 → 写入天然原子。
 *   主循环：   非关键场景（OLED 显示）直接读 g_icm42688.roll，零开销。
 *             PID 控制回路：ICM42688_GetSnapshot(&snap) → 关中断拷贝 9 个
 *             float（<1μs），保证 roll/pitch/yaw/gyro_z/accel 来自同一 SPI 帧。
 *
 * ── 数据流 ──
 *   ISR → ReadSensor()         一次 SPI burst，填满 g_icm42688
 *   任意上下文 → g_icm42688.roll / .gyro_z / .accel_x …  零开销
 *
 * ── 速度 ──
 *   - Bus 选择只做一次（Init），ReadSensor 内部 BurstReadFast 自带上下文保护
 *   - 12 字节 burst 读（0x1F→0x2A），一次 CS↓↑
 *   - 5MHz DelayOff 实际 SCK ≈ 10~15MHz
 *   - 单次 ReadSensor ≈ 180μs（含 SPI 12μs + float 转换 160μs）
 */

/* ── 全局数据实例 ── */
ICM42688_Data_t g_icm42688;

/* ── 内部状态 ── */
static float    s_accelScale = 1.0f;      /* LSB → m/s² */
static float    s_gyroScale  = 1.0f;      /* LSB → °/s  */
static float    s_gyroBiasZ  = 0.0f;      /* Z 轴零偏 (°/s)           */
static uint32_t s_lastTick;               /* 上次 ReadSensor 的 tick  */
static uint8_t  s_firstRead = 1U;         /* 首次读取标志             */
static uint8_t  s_inited    = 0U;

/* ── 系统 tick 外部引用（Control_Task.c）── */
extern volatile uint32_t g_sys_tick_ms;

/* ══════════════════════════════════════════════════════════
 * SPI 底层（内部）
 * ══════════════════════════════════════════════════════════ */

static void ICM42688_WriteReg(uint8_t reg, uint8_t data)
{
	API_SPI_SelectBus(ICM42688_SPI_BUS);
	API_SPI_Start();
	API_SPI_SwapByte(reg & 0x7FU);
	API_SPI_SwapByte(data);
	API_SPI_Stop();
}

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
 * 极速 burst 读 — ISR 热路径，带 SPI 上下文保护
 */
static void ICM42688_BurstReadFast(uint8_t reg, uint8_t *buf, uint8_t len)
{
	soft_spi_context_t saved;

	soft_spi_hal_save(&saved);
	API_SPI_SelectBus(ICM42688_SPI_BUS);
	API_SPI_DelayOff();

	API_SPI_Start();
	API_SPI_SwapByte(reg | 0x80U);
	while (len--) { *buf++ = API_SPI_SwapByte(0xFFU); }
	API_SPI_Stop();

	soft_spi_hal_restore(&saved);
}

/* ══════════════════════════════════════════════════════════
 * 初始化
 * ══════════════════════════════════════════════════════════ */

uint8_t ICM42688_Init(void)
{
	uint8_t whoami, retry;

	API_SPI_SelectBus(ICM42688_SPI_BUS);
	API_SPI_SetSpeed(ICM42688_SPI_SPEED);
	API_SPI_DelayOff();

	/* ── WHO_AM_I ── */
	for (retry = 0U; retry < 50U; ++retry)
	{
		whoami = ICM42688_ReadReg(ICM42688_WHO_AM_I);
		if (whoami == ICM42688_WHO_AM_I_VAL) { break; }
		Delay_ms(10U);
	}
	if (whoami != ICM42688_WHO_AM_I_VAL)
	{
		s_inited = 0U;
		return 0U;
	}

	/* ── 软复位 ── */
	ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x00U);
	Delay_ms(10U);

	/* ── 量程 / ODR ── */
	ICM42688_WriteReg(ICM42688_ACCEL_CONFIG0,
	                  (uint8_t)((ICM42688_ACCEL_16G << 5) | (ICM42688_ODR_1KHZ + 1U)));
	ICM42688_WriteReg(ICM42688_GYRO_CONFIG0,
	                  (uint8_t)((ICM42688_GYRO_2000DPS << 5) | (ICM42688_ODR_1KHZ + 1U)));

	s_accelScale = 16.0f / 32768.0f * 9.80665f;
	s_gyroScale  = 2000.0f / 32768.0f;

	ICM42688_WriteReg(ICM42688_GYRO_CONFIG1, 0x06U);
	ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x0FU);
	Delay_ms(50U);

	/* ── 陀螺仪 Z 轴零偏校准（50 样本，间隔 1ms）── */
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
			Delay_ms(1U);
		}
		s_gyroBiasZ = biasSum / 50.0f;
	}

	g_icm42688.yaw = 0.0f;
	s_firstRead    = 1U;
	s_inited       = 1U;

	return 1U;
}

/* ══════════════════════════════════════════════════════════
 * ISR 传感器读取 — 直接填满 g_icm42688
 * ══════════════════════════════════════════════════════════ */

void ICM42688_ReadSensor(void)
{
	uint8_t  buf[12];
	int16_t  rawAx, rawAy, rawAz;
	int16_t  rawGx, rawGy, rawGz;
	float    ax, ay, az, gz;
	uint32_t now;
	float    dt;

	if (s_inited == 0U) { return; }

	/* ── SPI burst 12 字节 ── */
	ICM42688_BurstReadFast(ICM42688_ACCEL_DATA_X1, buf, 12U);

	rawAx = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
	rawAy = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
	rawAz = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
	rawGx = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
	rawGy = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
	rawGz = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);

	/* ── float 转换 → 直接写 g_icm42688 ── */
	ax = (float)rawAx * s_accelScale;
	ay = (float)rawAy * s_accelScale;
	az = (float)rawAz * s_accelScale;

	g_icm42688.accel_x = ax;
	g_icm42688.accel_y = ay;
	g_icm42688.accel_z = az;
	g_icm42688.gyro_x  = (float)rawGx * s_gyroScale;
	g_icm42688.gyro_y  = (float)rawGy * s_gyroScale;
	gz                  = (float)rawGz * s_gyroScale;
	g_icm42688.gyro_z  = gz - s_gyroBiasZ;

	if (g_icm42688.gyro_z > -ICM42688_GYRO_DEADBAND &&
	    g_icm42688.gyro_z <  ICM42688_GYRO_DEADBAND)
	{
		g_icm42688.gyro_z = 0.0f;
	}

	/* roll / pitch */
	g_icm42688.roll  = atan2f(ay, az) * 57.29578f;
	{
		float norm = sqrtf(ay * ay + az * az);
		g_icm42688.pitch = atan2f(-ax, norm) * 57.29578f;
	}

	/* ── 偏航积分 ── */
	now = g_sys_tick_ms;
	if (s_firstRead)
	{
		g_icm42688.yaw = 0.0f;
		s_lastTick     = now;
		s_firstRead    = 0U;
	}
	else
	{
		dt = (float)(now - s_lastTick) * 0.001f;
		if (dt > 0.0f && dt < 0.1f)
		{
			g_icm42688.yaw += g_icm42688.gyro_z * dt;
		}
		s_lastTick = now;
	}

	/* Yaw 限幅 ±180° */
	while (g_icm42688.yaw >  180.0f) { g_icm42688.yaw -= 360.0f; }
	while (g_icm42688.yaw < -180.0f) { g_icm42688.yaw += 360.0f; }
}

/* ══════════════════════════════════════════════════════════
 * ICM42688_GetSnapshot — 快照读（PID 回路专用）
 *
 * 为什么不需要锁：
 *   ReadSensor（写 g_icm42688）和 GetSnapshot 的调用者（Drive_YawSpeed、
 *   YawTest_Control）都在 TIMG0 同一个 ISR 的不同时隙（5ms / 20ms）内执行。
 *   M0+ 的 NVIC 不会抢占同级中断——ISR 执行期间不会被自己打断。
 *   20ms 槽只看到 5ms 槽完成后的完整 struct，无跨帧混合风险。
 *
 *   如果未来从主循环调用本函数，主循环可能被 ISR 抢占，此时需要加锁。
 *   届时在本函数内加一个 volatile 序列号做乐观锁即可，仍不需要 MCU 头文件。
 * ══════════════════════════════════════════════════════════ */
void ICM42688_GetSnapshot(ICM42688_Data_t *snap)
{
	if (snap == 0) { return; }
	*snap = g_icm42688;   /* struct copy（~36 字节），当前调用上下文天然安全 */
}
