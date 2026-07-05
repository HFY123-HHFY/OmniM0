#ifndef __GRAY_ADC_H
#define __GRAY_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 运行模式切换
 *===========================================================================*/
/*
 * 1 = 校准模式：仅采集原始 ADC 值到 raw_value，不做二值化/归一化。
 *              用户通过串口读取 raw_value，确定 white/black 校准值。
 * 0 = 正常模式：使用校准参数，输出二值化(digital)和归一化(normalized)结果。
 */
#define GRAY_ADC_CALIBRATION_MODE  0U

/*===========================================================================
 * 校准参数（GRAY_ADC_CALIBRATION_MODE == 0 时使用）
 *===========================================================================*/
/*
 * 校准流程：
 *   1. 将 GRAY_ADC_CALIBRATION_MODE 改为 1，编译烧录
 *   2. 传感器放在**白色/浅色**区域，记录串口输出的 8 路 raw_value
 *   3. 传感器放在**黑色/深色**区域，记录串口输出的 8 路 raw_value
 *   4. 将记录的值填入下方两个数组，将 GRAY_ADC_CALIBRATION_MODE 改回 0
 *   5. 重新编译烧录，正常使用
 *
 * 注意：如果未调用 GrayADC_InitSensor()，驱动会在首次 Task 时自动使用默认值。
 */
#define GRAY_ADC_WHITE_DEFAULT  { 1800U, 1800U, 1800U, 1800U, \
                                  1800U, 1800U, 1800U, 1800U }
#define GRAY_ADC_BLACK_DEFAULT  {  300U,  300U,  300U,  300U, \
                                   300U,  300U,  300U,  300U }

/*===========================================================================
 * 配置结构体：地址选择引脚 — 由 Enroll 注册层填充
 *===========================================================================*/
typedef struct
{
    void     *ad0Port;   /* AD0 引脚端口 (PB17) */
    uint32_t  ad0Pin;    /* AD0 引脚编号       */
    void     *ad1Port;   /* AD1 引脚端口 (PB18) */
    uint32_t  ad1Pin;    /* AD1 引脚编号       */
    void     *ad2Port;   /* AD2 引脚端口 (PB19) */
    uint32_t  ad2Pin;    /* AD2 引脚编号       */
} GrayADC_Config_t;

/*===========================================================================
 * 传感器实例结构体 — 上层 App 持有和使用
 *===========================================================================*/
typedef struct
{
    /* ── 输出数据 ── */
    uint16_t raw_value[8];         /* 原始 ADC 值（8 路），校准/正常模式均有效 */
    uint16_t normalized[8];        /* 归一化值 [0 ~ bits]，仅正常模式有效   */
    uint8_t  digital;              /* 二值化结果 bit0=第1路...bit7=第8路    */

    /* ── 校准参数（由 GrayADC_InitSensor 填充） ── */
    uint16_t calib_white[8];       /* 白色基准 ADC 值              */
    uint16_t calib_black[8];       /* 黑色基准 ADC 值              */
    uint16_t threshold_white[8];   /* 白阈值 = (white*2+black)/3   */
    uint16_t threshold_black[8];   /* 黑阈值 = (white+black*2)/3   */
    double   norm_factor[8];       /* 归一化系数 = bits/(w-b)      */

    double   bits;                 /* ADC 量程（12-bit = 4096.0）  */
    uint8_t  calib_ready;          /* 1 = 校准已就绪               */
} GrayADC_Sensor_t;

/*===========================================================================
 * API 函数声明
 *===========================================================================*/

/*
 * 注册 GrayADC 地址引脚配置表。
 * 由 Enroll_GrayADC_Register() 调用，App 层无需关心。
 */
void GrayADC_Register(const GrayADC_Config_t *configTable, uint8_t count);

/*
 * 初始化地址选择引脚（AD0/AD1/AD2）为推挽输出，默认选中通道 0。
 * 必须在 GrayADC_Register() 之后调用。
 */
void GrayADC_Init(void);

/*
 * 选通传感器通道（channel: 0~7）。
 * 通过 74HC4051 的 AD0/AD1/AD2 地址线切换模拟开关，并等待 1us 稳定。
 */
void GrayADC_SelectChannel(uint8_t channel);

/*
 * 读取 8 路传感器的原始 ADC 值（每路 8 次过采样取均值）。
 * 结果存入 sensor->raw_value[0..7]。
 */
void GrayADC_ReadAllRaw(GrayADC_Sensor_t *sensor);

/*
 * 传感器校准初始化。
 *
 * 参数：
 *   calib_white[8] — 传感器位于白色/浅色表面时采集的 8 路 ADC 均值
 *   calib_black[8] — 传感器位于黑色/深色表面时采集的 8 路 ADC 均值
 *
 * 内部自动计算：
 *   - 二值化阈值（1:2 / 2:1 分界）
 *   - 归一化系数（映射到 0 ~ bits）
 *   - 标记 calib_ready = 1
 *
 * 如果不想手动调用，可将校准值填入 GRAY_ADC_WHITE/BLACK_DEFAULT 宏，
 * 驱动会在首次 GrayADC_Task() 时自动使用默认值初始化。
 */
void GrayADC_InitSensor(GrayADC_Sensor_t *sensor,
                        const uint16_t *calib_white,
                        const uint16_t *calib_black);

/*
 * 传感器主任务（每个控制周期调用一次）。
 *
 * 校准模式 (GRAY_ADC_CALIBRATION_MODE == 1)：
 *   仅采集 raw_value，用户通过串口打印观察。
 *
 * 正常模式 (GRAY_ADC_CALIBRATION_MODE == 0)：
 *   采集 raw_value → 二值化 → 归一化，完整输出。
 */
void GrayADC_Task(GrayADC_Sensor_t *sensor);

/*
 * 获取二值化结果（8-bit，bit0 = 第1路）：
 *   1 = 亮（白色），0 = 暗（黑色）。
 * 仅在正常模式且校准完成后有意义。
 */
uint8_t GrayADC_GetDigital(const GrayADC_Sensor_t *sensor);

/*
 * 获取归一化结果数组指针（8 个 uint16_t）。
 * 仅在正常模式且校准完成后有效；否则返回 NULL。
 */
const uint16_t *GrayADC_GetNormalized(const GrayADC_Sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* __GRAY_ADC_H */
