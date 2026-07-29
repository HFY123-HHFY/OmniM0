#ifndef __YABO_IR_H
#define __YABO_IR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 传感器物理参数（根据实际硬件调整）
 *===========================================================================*/

/*
 * 两个传感器管之间的中心间距（毫米）。
 * 模块上 8 路探头均匀排列，默认间距 12mm。
 * 例：12mm → 位置范围 [0, 8400]，居中 = 4200
 */
#define YABO_IR_SENSOR_SPACING_MM  12U

/*
 * 位置输出 EMA 低通滤波强度（0 = 不滤波）。
 *
 * 值越大滤波越重、位置越稳、但响应越慢。
 * 建议：2~6（轻滤波），比赛时如果赛道抖动大可以调到 8~12。
 * 设为 0 则完全关闭滤波，raw 位置直出。
 */
#define YABO_IR_POSITION_SMOOTHING  4U

/*
 * 数字量极性取反开关。
 *
 * 亚博模块原始数据：1 = 黑线（探到），0 = 白（未探到）
 * GrayADC 惯例：       0 = 黑线（在线），1 = 白（离线）
 *
 * 设为 1：自动取反，使 digital_bits[] 与 GrayADC 惯例一致。
 * 设为 0：保留模块原始值，x:1→1, x:0→0。
 *
 * 建议保持默认 1（匹配 GrayADC / PID 调用约定）。
 */
#define YABO_IR_INVERT_DIGITAL     1U

/*===========================================================================
 * 传感器实例结构体 — 上层 App 持有和使用
 *===========================================================================*/

typedef struct
{
    /* ── 模块原始数据 ── */
    uint8_t  raw_bits[8];          /* 模块直接上报：0=白, 1=黑（见手册寄存器 0x30） */

    /* ── 转换后数据（GrayADC 兼容格式）── */
    uint8_t  digital_bits[8];      /* 二值化：0=黑（在线）, 1=白（离线）          */
    uint8_t  digital;              /* 8 路合并：bit0=S0 ... bit7=S7（0=黑,1=白）  */

    /* ── 位置计算 ── */
    int32_t  pos_filtered;         /* LinePosition EMA 滤波状态                   */
    uint8_t  data_ready;           /* 1 = 至少收到过一帧有效数据                   */
    uint8_t  frame_updated;        /* 1 = 本周期收到新帧（Task 置位，上层清零）     */
} YaboIR_Sensor_t;

/*===========================================================================
 * API 函数声明
 *===========================================================================*/

/*
 * 初始化传感器结构体（清零所有字段，设置默认滤波状态）。
 * 硬件（USART4）由 main.c 统一初始化，本模块不操作寄存器。
 */
void YaboIR_Init(YaboIR_Sensor_t *sensor);

/*
 * 向模块发送初始化命令 "$0,0,1#"（仅数字量输出模式）。
 *
 * 调用时机：USART4 已初始化、TIMG0 启动之前（main.c Init 阶段）。
 * 会阻塞约 300µs（6 字节 @115200bps ≈ 520µs + TX FIFO 排空）。
 * 发送完成后模块立即开始持续上报 "$D,...#" 帧。
 */
void YaboIR_SendCmd(void);

/*
 * 推入 1 字节到环形缓冲（中断上半部，USART4 ISR 调用）。
 *
 * 操作极轻：数组写入 + 索引自增，< 1µs @80MHz。
 * 缓冲满时丢弃最老字节（滑动窗口，防止死锁）。
 */
void YaboIR_RxPush(uint8_t data);

/*
 * 传感器主任务 — 在 TIMG0 ISR 5ms 槽中调用。
 *
 * 从环形缓冲取字节 → 状态机解析 "$D,...#" 帧 → 更新 sensor 成员。
 * 同时自动完成 raw→digital 取反（由 YABO_IR_INVERT_DIGITAL 控制）。
 *
 * 耗时：典型 < 10µs（8 次 strchr + 赋值），ISR 安全。
 */
void YaboIR_Task(YaboIR_Sensor_t *sensor);

/*
 * 打印 8 路二值化 bits（纯 0/1），不包含原始值。
 *
 * 输出格式：D:00111100
 *   - 0 = 黑线（在线），1 = 白线（离线）
 *   - 例：00111100 表示 S2~S5 看到黑线
 *
 * 用法：YaboIR_PrintBits(&g_yaboIR, USART1);
 */
void YaboIR_PrintBits(const YaboIR_Sensor_t *sensor, void *usart);

/*
 * 打印线位置 + 偏差 + 二值化 bits（PID 调试专用）。
 *
 * 输出格式：POS:4200 E:-120 D:00111100
 *   POS = 黑线加权位置 (0~8400 @12mm)，EMA 滤波后
 *   E   = 偏差 (POS - 中心)，负=偏左，正=偏右
 *   D   = 8 路二值化状态
 *
 * 用法：YaboIR_PrintLinePos(&g_yaboIR, USART1);
 */
void YaboIR_PrintLinePos(YaboIR_Sensor_t *sensor, void *usart);

/*
 * 计算黑线位置 — 加权平均法 + EMA 低通滤波。
 *
 * 原理：
 *   传感器排列（假设间距 12mm，8 路从左到右）：
 *    [S0] [S1] [S2] [S3] [S4] [S5] [S6] [S7]
 *     0   1200  2400  3600  4800  6000  7200  8400  ← mm × 100
 *
 *   对于数字型传感器，黑线覆盖的探头 digital_bits[] = 0，
 *   位置 = 所有黑线探头的中心加权平均。
 *
 *   加权公式：
 *     pos = Σ( weight[i] * i * spacing * 100 ) / Σ( weight[i] )
 *     weight[i] = (digital_bits[i] == 0) ? 1 : 0
 *
 *   再经 EMA 低通：
 *     filtered = filtered*(1-1/N) + rawPos*(1/N)
 *
 *   丢线保护：全白（无探头看到黑线）时保持上一次有效位置。
 *
 * 返回值：
 *   [0, 7*spacing*100] — 黑线加权中心位置（单位 = 0.01mm）
 *   -1                 — sensor 无效/从未收到数据
 *
 * 典型巡线 PID 用法：
 *   int32_t pos   = YaboIR_LinePosition(&g_yaboIR);
 *   int32_t error = pos - (7*间距*100/2);
 *   int32_t steer = PID_Direction_Calculate(error);
 */
int32_t YaboIR_LinePosition(YaboIR_Sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* __YABO_IR_H */
