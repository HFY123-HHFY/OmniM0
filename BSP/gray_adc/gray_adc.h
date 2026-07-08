#ifndef __GRAY_ADC_H
#define __GRAY_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 传感器物理参数（根据实际硬件调整）
 *===========================================================================*/

/*
 * 两个传感器管之间的中心间距（毫米）。
 * 用直尺量相邻两个灰度管的中心距离，填入实际值。
 * 例：12mm → 位置范围 [0, 8400]，居中 = 4200
 */
#define GRAY_ADC_SENSOR_SPACING_MM  12U

/*
 * 位置输出 EMA 低通滤波强度（0 = 不滤波）。
 *
 * 值越大滤波越重、位置越稳、但响应越慢。
 * 建议：2~6（轻滤波），比赛时如果赛道抖动大可以调到 8~12。
 * 设为 0 则完全关闭滤波，raw 位置直出。
 */
#define GRAY_ADC_POSITION_SMOOTHING  4U

/*===========================================================================
 * 校准参数
 *===========================================================================*/
/*
 * 校准流程：
 *   1. 传感器放在**白色/浅色**区域，用 GrayADC_PrintRaw() 打印 8 路 raw_value
 *   2. 传感器放在**黑色/深色**区域，用 GrayADC_PrintRaw() 打印 8 路 raw_value
 *   3. 将记录的值填入下方两个数组，重新编译烧录即可
 *
 * 注意：如果未调用 GrayADC_InitSensor()，驱动会在首次 Task 时自动使用默认值。
 */
/* 顺序已翻转为 sensor[0]=最右路, sensor[7]=最左路 */
#define GRAY_ADC_WHITE_DEFAULT  { 3335U, 3339U, 3091U, 3307U, \
                                  3279U, 3336U, 2383U, 2347U }
#define GRAY_ADC_BLACK_DEFAULT  {   88U,   94U,   93U,   97U, \
                                    99U,   90U,   94U,   96U }

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
    uint8_t  digital_bits[8];      /* 二值化拆分：digital_bits[0]=第1路(0/1), ...[7]=第8路 */

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
 * 完整流程：采集 raw_value → 二值化 → 归一化。
 * 如果尚未调用 GrayADC_InitSensor()，自动使用默认校准值初始化一次。
 */
void GrayADC_Task(GrayADC_Sensor_t *sensor);

/*
 * 打印 8 路原始 ADC 值（用于校准）。
 *
 * 输出格式：RAW: 2100 2050 1980 2120 1500 2080 2030 2070
 *
 * 用法：GrayADC_PrintRaw(&g_graySensor, USART1);
 * 用于校准模式下观察原始值，确定 white/black 校准参数。
 */
void GrayADC_PrintRaw(const GrayADC_Sensor_t *sensor, void *usart);

/*
 * 打印 8 路二值化 bits（纯 0/1），不包含 ADC 原始值。
 *
 * 输出格式：D:00111100
 *   - 0 = 黑线（暗），1 = 白线（亮）
 *   - 例：00111100 表示第3、4、5、6路看到黑线
 *
 * 用法：GrayADC_PrintBits(&g_graySensor, USART1);
 */
void GrayADC_PrintBits(const GrayADC_Sensor_t *sensor, void *usart);

/*
 * 打印线位置 + 偏差 + 二值化 bits（PID 调试专用）。
 *
 * 输出格式：POS:4200 E:-120 D:00111100
 *   POS = 黑线加权位置 (0~8400 @12mm)，EMA 滤波后
 *   E   = 偏差 (POS - 中心)，负=偏左，正=偏右
 *   D   = 8 路二值化状态
 *
 * 用法：GrayADC_PrintLinePos(&g_graySensor, USART1);
 */
void GrayADC_PrintLinePos(const GrayADC_Sensor_t *sensor, void *usart);

/*
 * 计算黑线位置 — 加权平均法 + EMA 低通滤波。
 *
 * 返回值：黑线的加权中心位置（单位 = 0.01mm）。
 *   范围 [0, spacing×700]，居中 = spacing×700/2。
 *   例 12mm 间距 → [0, 8400]，居中 = 4200。
 *   返回 -1 = 传感器无效/未校准。
 *
 * 位置分辨率（加权平均自然提供亚传感器精度）：
 *   黑线覆盖 ≥2 个传感器时，归一化值产生中间权重，
 *   位置会平滑过渡而非跳变。
 *   例：线从 S3 滑到 S4，pos 从 ~3600 渐渐变到 ~4800，约 1200 个细分步进。
 *
 * EMA 滤波（GRAY_ADC_POSITION_SMOOTHING 控制强度）：
 *   filtered = filtered*(1-1/N) + raw*(1/N)
 *   滤除单次 ADC 噪声和微小机械抖动，让 PID 输入更干净。
 *
 * 丢线保护：全白时保持上一次有效位置，不会乱跳。
 *
 * 典型巡线 PID 用法：
 *   int32_t pos   = GrayADC_LinePosition(&g_graySensor);
 *   int32_t error = pos - (7*间距*100/2);  // 减中心
 *   int32_t steer = PID_Direction_Calculate(error);
 *   TB6612_SetSpeed(baseSpeed + steer, baseSpeed - steer);
 */
int32_t GrayADC_LinePosition(const GrayADC_Sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* __GRAY_ADC_H */
