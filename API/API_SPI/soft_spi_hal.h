#ifndef __SOFT_SPI_HAL_H
#define __SOFT_SPI_HAL_H

#include <stdint.h>

/*
 * soft_spi_hal.h — 软件 SPI 硬件抽象层接口 (内部桥接头文件)
 *
 * 定位: API 协议层 ↔ Core 底层实现 之间的内部桥梁。
 *       BSP/App 层不应直接引用本头文件，应使用 API_SPI.h。
 *
 * 职责: 声明平台无关的 GPIO 翻转与延时原语。
 * 实现: 由 Core/{platform}/{platform}_soft_spi.c 按平台提供。
 *
 * 本头文件不含任何平台条件编译。
 */

/* GPIO 引脚一次性初始化并预计算全部寄存器缓存值。 */
void soft_spi_hal_init(void *csPort, uint32_t csPin, uint32_t csIomux,
                       void *sckPort, uint32_t sckPin, uint32_t sckIomux,
                       void *mosiPort, uint32_t mosiPin, uint32_t mosiIomux,
                       void *misoPort, uint32_t misoPin, uint32_t misoIomux);

/* 写 CS/SCK/MOSI 电平 (0 或非 0)。 */
void soft_spi_hal_w_cs(uint8_t bit);
void soft_spi_hal_w_sck(uint8_t bit);
void soft_spi_hal_w_mosi(uint8_t bit);

/* 读 MISO 电平, 返回 0 或 1。 */
uint8_t soft_spi_hal_r_miso(void);

/* 基础延时 (us), 内部检查延时关闭标志。 */
void soft_spi_hal_delay_us(uint32_t us);

/* 速率档位预计算。speedKhz: 250/500/1000/2000/5000。 */
void soft_spi_hal_set_speed(uint32_t speedKhz);
/* 关闭/恢复 bit-bang 延时。 */
void soft_spi_hal_delay_off(void);
void soft_spi_hal_delay_on(void);

/* ══════════════════════════════════════════════════════════
 * SPI 上下文保存/恢复（ISR 抢占保护）
 * ══════════════════════════════════════════════════════════
 *
 * 当 ISR（如 ICM42688 @5ms TIMG0）需要抢占主循环的 SPI 事务时：
 *   1. ISR 调用 soft_spi_hal_save() 保存当前总线上下文
 *   2. ISR 切换到自己的总线并完成 SPI 事务
 *   3. ISR 调用 soft_spi_hal_restore() 恢复主循环的总线上下文
 *
 * 这确保了两路 SPI 总线的 GPIO 操作互不干扰，
 * 且主循环的事务在 ISR 返回后能无缝继续。
 */

/* SPI 上下文：保存当前总线的全部 GPIO 寄存器指针和延时参数。 */
typedef struct
{
	void  *csReg;
	void  *sckReg;
	void  *mosiReg;
	void  *misoReg;
	uint32_t csPin;
	uint32_t sckPin;
	uint32_t mosiPin;
	uint32_t misoPin;
	uint8_t  delayUs;
	uint8_t  delayOff;
} soft_spi_context_t;

/* 保存当前软件 SPI 上下文到 ctx。 */
void soft_spi_hal_save(soft_spi_context_t *ctx);
/* 从 ctx 恢复软件 SPI 上下文。 */
void soft_spi_hal_restore(const soft_spi_context_t *ctx);

#endif /* __SOFT_SPI_HAL_H */
