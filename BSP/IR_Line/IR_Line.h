#ifndef __IR_LINE_H
#define __IR_LINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 幻尔 八路红外巡线模块 — ADC 直读驱动
 *
 * 模块自带 MCU + 校准按键，软件侧无需重复校准。
 * 74HC4051 模拟开关 + 8 路红外对管 → G3507 ADC1 CH0。
 * GPIO/ADC 与 GrayADC 物理复用，由 GrayADC_Init 统一初始化。
 *
 * 校准流程：按下模块上的校准按键 → 模块 MCU 自动调整各通道一致性 →
 *          各通道黑白电平趋于均匀 → G3507 侧固定阈值二值化即可。
 *===========================================================================*/

/*
 * 两个探头之间的中心间距（毫米）。
 * 例 12mm → 8 路位置范围 [0, 8400]（单位 0.01mm），居中 = 4200。
 */
#define IRLINE_SENSOR_SPACING_MM  12U

/*
 * 位置输出 EMA 低通滤波强度（0 = 不滤波）。
 * 值越大越稳但响应越慢，建议 2~6。
 */
#define IRLINE_POSITION_SMOOTHING  4U

/*
 * 固定二值化阈值（12-bit ADC，量程 0~4095）。
 *
 * 模块按键校准后，各通道黑/白电平趋于一致。
 * 此阈值用于区分黑线和白线：
 *   ADC 值 > 阈值 → 判为黑线 (digital_bits = 1)
 *   ADC 值 ≤ 阈值 → 判为白线 (digital_bits = 0)
 *
 * 阈值不会覆盖模块按键校准结果——校准管的是传感器硬件一致性，
 * 阈值管的是软件二值化分界线，两者各司其职。
 *
 * 实测调整方法：
 *   1. 模块放白纸上，串口打印 raw_value，取平均值 ≈ white
 *   2. 模块放黑线上，串口打印 raw_value，取平均值 ≈ black
 *   3. 阈值设为 (white + black) / 2
 */
#define IRLINE_THRESHOLD  1500U

/*===========================================================================
 * 传感器实例结构体
 *===========================================================================*/
typedef struct
{
    uint16_t raw_value[8];         /* 8 路原始 ADC 值（12-bit）        */
    uint8_t  digital_bits[8];      /* 二值化结果：1=黑线, 0=白线      */
    uint8_t  digital;              /* 合并字节：bit0=S0 ... bit7=S7   */
    int32_t  pos_filtered;         /* LinePosition EMA 滤波状态        */
    uint16_t min_adc[8];           /* 白底基线（运行最小值跟踪），扣除后白底权重→0 */
    uint8_t  data_ready;           /* 1 = 至少采集过一次有效数据       */
    uint8_t  frame_updated;        /* 1 = 本周期有新数据（Task 置位）  */
} IRLine_Sensor_t;

extern IRLine_Sensor_t g_irLine;

/*===========================================================================
 * API 函数声明
 *===========================================================================*/

/*
 * 初始化传感器结构体（清零所有字段，设置默认滤波状态）。
 * 注意：GPIO/ADC 硬件由 GrayADC_Init 统一初始化，本函数不操作寄存器。
 * 调用时机：必须在 GrayADC_Init 之后、TIMG0 启动前。
 */
void IRLine_Init(IRLine_Sensor_t *sensor);

/*
 * 传感器主任务 — 在 TIMG0 ISR 5ms 槽中调用。
 *
 * 完整流程：
 *   1. 遍历 8 通道，GrayADC_SelectChannel 选通
 *   2. 每通道 4 次 ADC 过采样取均值 → raw_value[0..7]
 *   3. 固定阈值二值化 → digital_bits[0..7]
 */
void IRLine_Task(IRLine_Sensor_t *sensor);

/*
 * 打印 8 路二值化 bits（纯 0/1）。
 * 输出格式：D:00111100
 *   1 = 黑线（探到），0 = 白线（未探到）
 */
void IRLine_PrintRaw(const IRLine_Sensor_t *sensor, void *usart);
void IRLine_PrintBits(const IRLine_Sensor_t *sensor, void *usart);

/*
 * 打印线位置 + 偏差 + 二值化 — PID 调参专用，一屏看全。
 *
 * 输出示例（12mm 间距）：POS:4200 E:-120 D:00111100
 *   POS = 黑线加权位置 (0~8400，单位 0.01mm)
 *   E   = 偏差 (POS - 中心 4200)，负=偏左，正=偏右
 *   D   = 8 路二值化状态
 */
void IRLine_PrintLinePos(IRLine_Sensor_t *sensor, void *usart);

/*
 * 计算黑线位置 — 连续灰度加权平均法 + EMA 低通滤波。
 *
 * 传感器排列（间距 12mm，8 路从左到右）：
 *   [S0]  [S1]  [S2]  [S3]  [S4]  [S5]  [S6]  [S7]
 *     0   1200  2400  3600  4800  6000  7200  8400  ← mm × 100
 *
 * 权重 = raw_value[i]（连续 ADC 值），越黑权重越大。
 * 丢线保护：全白时保持上一次有效位置。
 *
 * 返回值：
 *   [0, 7×spacing×100] — 黑线加权中心位置（单位 0.01mm）
 *   -1                 — sensor 无效 / 从未收到数据
 *
 * 典型 PID 用法：
 *   int32_t pos   = IRLine_LinePosition(&g_irLine);
 *   int32_t error = pos - (7 * spacing * 100 / 2);
 *   g_steer = PID_Calc(&direction_pid, pos);
 */
int32_t IRLine_LinePosition(IRLine_Sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* __IR_LINE_H */
