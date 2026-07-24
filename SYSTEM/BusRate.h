#ifndef __BUS_RATE_H
#define __BUS_RATE_H

#include "API_I2C.h"
#include "API_SPI.h"

/*
 * BusRate.h — 软件总线统一配置中心
 *
 * 职责:
 * - 集中管理所有 BSP 设备的 I2C/SPI 总线选择（用哪一路总线）
 * - 集中管理所有 BSP 设备的 I2C/SPI 速率档位（跑多快）
 * - 按 MCU 目标区分，一个文件看清全部总线策略
 *
 * 速度说明（80MHz 主频 + DelayOff）：
 *   5MHz 档位：实际 SCK ≈ 10~15MHz（GPIO 翻转极限）
 *   SPI1（OLED）和 SPI2（ICM42688）同为 5MHz，无切换开销
 *
 * 维护原则:
 * - 新增 I2C/SPI 设备时，总线选择与速率档位都在这里定义
 * - 不要在 BSP 驱动的 .h 文件中单独定义总线选择
 * - 调速/换总线只需改这一个文件
 */

/* ── 总线选择 ── */
#define OLED_I2C_BUS           API_I2C2
#define OLED_SPI_BUS           API_SPI1
#define MPU6050_I2C_BUS        API_I2C1
#define ICM42688_SPI_BUS       API_SPI2      /* ICM42688 陀螺仪 */

/* ── 速率档位 ── */
#define OLED_I2C_SPEED          API_I2C_SPEED_400K
#define MPU6050_I2C_SPEED       API_I2C_SPEED_400K
#define OLED_SPI_SPEED          API_SPI_SPEED_5M
#define ICM42688_SPI_SPEED      API_SPI_SPEED_5M   /* 陀螺仪跑满，OLED 同速无切换开销 */

#endif /* __BUS_RATE_H */
