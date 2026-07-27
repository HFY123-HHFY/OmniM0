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
 *   ISR → ICM42688_ReadSensor()      // 一次 SPI burst，填满 g_icm42688
 *   任意上下文 → g_icm42688.roll     // 直接读结构体字段，零开销
 */

/* ══════════════════════════════════════════════════════════
 * 寄存器 Bank 0
 * ══════════════════════════════════════════════════════════ */
#define ICM42688_GYRO_DEADBAND  (0.25f)  /* °/s, 低于此阈值的角速度视为静止     */

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
    ICM42688_ACCEL_16G   = 0,
    ICM42688_ACCEL_8G    = 1,
    ICM42688_ACCEL_4G    = 2,
    ICM42688_ACCEL_2G    = 3,
} ICM42688_AccelRange_t;

typedef enum
{
    ICM42688_GYRO_2000DPS  = 0,
    ICM42688_GYRO_1000DPS  = 1,
    ICM42688_GYRO_500DPS   = 2,
    ICM42688_GYRO_250DPS   = 3,
    ICM42688_GYRO_125DPS   = 4,
    ICM42688_GYRO_62_5DPS  = 5,
    ICM42688_GYRO_31_25DPS = 6,
    ICM42688_GYRO_15_625DPS= 7,
} ICM42688_GyroRange_t;

typedef enum
{
    ICM42688_ODR_32KHZ = 0,
    ICM42688_ODR_16KHZ = 1,
    ICM42688_ODR_8KHZ  = 2,
    ICM42688_ODR_4KHZ  = 3,
    ICM42688_ODR_2KHZ  = 4,
    ICM42688_ODR_1KHZ  = 5,
    ICM42688_ODR_200HZ = 6,
    ICM42688_ODR_100HZ = 7,
} ICM42688_Odr_t;

/* ══════════════════════════════════════════════════════════
 * 全局数据（ReadSensor 直接填充，任意上下文零开销读取）
 * ══════════════════════════════════════════════════════════ */
typedef struct
{
    float accel_x;     /* 加速度 X (m/s²) */
    float accel_y;     /* 加速度 Y (m/s²) */
    float accel_z;     /* 加速度 Z (m/s²) */
    float gyro_x;      /* 角速度 X (°/s)  */
    float gyro_y;      /* 角速度 Y (°/s)  */
    float gyro_z;      /* 角速度 Z (°/s)  */
    float roll;        /* 横滚角 (°)      — 由加速度反算 */
    float pitch;       /* 俯仰角 (°)      — 由加速度反算 */
    float yaw;         /* 偏航角 (°, ±180) — gyro_z 积分   */
} ICM42688_Data_t;

extern ICM42688_Data_t g_icm42688;

/*
 * ICM42688_GetSnapshot — 原子快照（供 PID 控制回路使用）
 *
 * 关中断拷贝 g_icm42688 的全部字段到 *snap（< 1μs），
 * 保证 roll/pitch/yaw/gyro_z 等字段来自同一帧 SPI 数据。
 *
 * 非关键场景（LED/OLED 显示）可直接读 g_icm42688，无需快照。
 */
void ICM42688_GetSnapshot(ICM42688_Data_t *snap);

/* ══════════════════════════════════════════════════════════
 * API
 * ══════════════════════════════════════════════════════════ */
uint8_t ICM42688_Init(void);
void    ICM42688_ReadSensor(void);   /* ISR 调用：填满 g_icm42688 */

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H__ */
