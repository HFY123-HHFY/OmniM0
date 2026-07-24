#ifndef __ICM42688_H
#define __ICM42688_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ICM42688 — TDK ICM-42688-P 6 轴 IMU 驱动（软件 SPI2, 极速模式）
 *
 * 硬件连接（SPI2）：
 *   SCK  → PA25    MOSI → PB25
 *   MISO → PA24    CS   → PB23
 *
 * 通信速率：5MHz DelayOff（80MHz 主频下实际 SCK ≈ 10~15MHz）
 * 默认配置：±16g / ±2000dps / 陀螺仪1kHz+加速度计1kHz / 低噪声
 *
 * 调用模型（ISR 驱动）：
 *   ISR → ICM42688_ReadSensor()          // 一次 SPI burst，全缓存（含偏航积分）
 *   主循环 → ICM42688_GetAttitude/Gyroscope/Accelerometer()  // 只读缓存，零 SPI
 */

/* ══════════════════════════════════════════════════════════
 * 寄存器 Bank 0
 * ══════════════════════════════════════════════════════════ */
#define ICM42688_DEVICE_CONFIG         0x11
#define ICM42688_TEMP_DATA1            0x1D
#define ICM42688_TEMP_DATA0            0x1E
#define ICM42688_ACCEL_DATA_X1         0x1F
#define ICM42688_ACCEL_DATA_X0         0x20
#define ICM42688_ACCEL_DATA_Y1         0x21
#define ICM42688_ACCEL_DATA_Y0         0x22
#define ICM42688_ACCEL_DATA_Z1         0x23
#define ICM42688_ACCEL_DATA_Z0         0x24
#define ICM42688_GYRO_DATA_X1          0x25
#define ICM42688_GYRO_DATA_X0          0x26
#define ICM42688_GYRO_DATA_Y1          0x27
#define ICM42688_GYRO_DATA_Y0          0x28
#define ICM42688_GYRO_DATA_Z1          0x29
#define ICM42688_GYRO_DATA_Z0          0x2A
#define ICM42688_PWR_MGMT0             0x4E
#define ICM42688_GYRO_CONFIG0          0x4F
#define ICM42688_ACCEL_CONFIG0         0x50
#define ICM42688_GYRO_CONFIG1          0x51
#define ICM42688_WHO_AM_I              0x75
#define ICM42688_REG_BANK_SEL          0x76

#define ICM42688_WHO_AM_I_VAL          0x47U

/* ── 量程 ── */
typedef enum
{
    ICM42688_ACCEL_16G   = 0,          /* ±16g (default) */
    ICM42688_ACCEL_8G    = 1,
    ICM42688_ACCEL_4G    = 2,
    ICM42688_ACCEL_2G    = 3,
} ICM42688_AccelRange_t;

typedef enum
{
    ICM42688_GYRO_2000DPS  = 0,        /* ±2000°/s (default)  */
    ICM42688_GYRO_1000DPS  = 1,
    ICM42688_GYRO_500DPS   = 2,
    ICM42688_GYRO_250DPS   = 3,
    ICM42688_GYRO_125DPS   = 4,
    ICM42688_GYRO_62_5DPS  = 5,
    ICM42688_GYRO_31_25DPS = 6,
    ICM42688_GYRO_15_625DPS= 7,
} ICM42688_GyroRange_t;

/* ── ODR ── */
typedef enum
{
    ICM42688_ODR_32KHZ = 0,            /* 32kHz  */
    ICM42688_ODR_16KHZ = 1,            /* 16kHz  */
    ICM42688_ODR_8KHZ  = 2,            /* 8kHz   */
    ICM42688_ODR_4KHZ  = 3,            /* 4kHz   */
    ICM42688_ODR_2KHZ  = 4,            /* 2kHz   */
    ICM42688_ODR_1KHZ  = 5,            /* 1kHz (default) */
    ICM42688_ODR_200HZ = 6,            /* 200Hz  */
    ICM42688_ODR_100HZ = 7,            /* 100Hz  */
} ICM42688_Odr_t;

/* ══════════════════════════════════════════════════════════
 * 用户 API
 * ══════════════════════════════════════════════════════════ */

/*
 * ICM42688_Init — 初始化（WHO_AM_I 校验 + 软复位 + 配置 + 上电）
 *
 * 需在 API_SPI_Init() 之后调用。
 * 失败死循环。
 */
void ICM42688_Init(void);

/*
 * ICM42688_ReadSensor — 一次 SPI burst 读取全部传感器数据（供 ISR 调用）
 *
 * 操作（按顺序）：
 *   1. SPI burst 12 字节（ACCEL_X1 → GYRO_Z0）
 *   2. float 转换 + roll/pitch 反算
 *   3. 偏航积分：yaw += gyro_z * dt（dt = 距离上次调用的秒数）
 *
 * 含浮点运算（atan2f, sqrtf），在 M0+ ISR 中约需 100~200μs。
 * 推荐调用频率 ≥ ICM42688 ODR（默认 1kHz → 每 1ms 一次）。
 *
 * 偏航积分需要 dt —— 使用系统 tick (g_sys_tick_ms) 自动计算，
 * 首次调用不积分（仅记录时间戳）。
 */
void ICM42688_ReadSensor(void);

/*
 * ICM42688_GetAttitude — 读姿态角（°）
 *   roll  : 横滚角 -180~180
 *   pitch : 俯仰角 -90~90
 *   yaw   : 偏航角（积分值，无边界，首次调用后从 0 开始）
 *
 * 数据来源：上一次 ICM42688_ReadSensor() 的缓存，零 SPI。
 */
void ICM42688_GetAttitude(float *roll, float *pitch, float *yaw);

/*
 * ICM42688_GetGyroscope — 读角速度（°/s）
 *   gx, gy, gz : 三轴角速度
 */
void ICM42688_GetGyroscope(float *gx, float *gy, float *gz);

/*
 * ICM42688_GetAccelerometer — 读加速度（m/s²）
 *   ax, ay, az : 三轴加速度
 */
void ICM42688_GetAccelerometer(float *ax, float *ay, float *az);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H__ */
