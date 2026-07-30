/*
 * IR_Line.c — 幻尔 八路红外巡线模块 ADC 直读驱动
 *
 * 硬件背景：
 *   - 模块自带 MCU + 校准按键，软件侧无需重复校准
 *   - 74HC4051 模拟开关 + 8 路红外对管
 *   - OUT 引脚 → G3507 ADC1 CH0（PB20）
 *
 * 架构约定（遵循 OmniM0 分层规范）：
 *   - GPIO/ADC 与 GrayADC 物理复用，由 GrayADC_Init 统一初始化
 *   - 通道切换复用 GrayADC_SelectChannel()
 *   - ADC 读取通过 API_ADC_GetValue()
 *   - BSP 层不直接写寄存器
 */

#include "IR_Line.h"
#include "gray_adc.h"           /* GrayADC_SelectChannel */
#include "adc.h"                /* API_ADC_GetValue */
#include "My_Usart/My_Usart.h"  /* usart_printf */

/* ADC 实例与通道，与 GrayADC 保持一致 */
#define ADC_INST   API_ADC1
#define ADC_CH     API_ADC_CH0

/* 每通道过采样次数（均值滤波，抑制电源噪声） */
#define SAMPLES    4U

/*===========================================================================
 * 全局传感器实例
 *===========================================================================*/
IRLine_Sensor_t g_irLine;

/*===========================================================================
 * IRLine_Init — 初始化传感器结构体
 *
 * 仅清零结构体字段 + 设置默认 EMA 滤波中心位置。
 * GPIO 三根地址线（AD0/AD1/AD2）由 GrayADC_Init 已配置为推挽输出，
 * 本函数不重复操作硬件，避免覆盖 GrayADC 的初始化状态。
 *
 * 调用时机：GrayADC_Init 之后、TIMG0 启动之前。
 *===========================================================================*/
void IRLine_Init(IRLine_Sensor_t *sensor)
{
    uint8_t i;

    if (sensor == 0) { return; }

    /* ── 所有字段清零 ── */
    for (i = 0U; i < 8U; i++)
    {
        sensor->raw_value[i]    = 0U;
        sensor->digital_bits[i] = 0U;  /* 默认白线（未探到黑线） */
    }
    sensor->digital       = 0U;
    sensor->pos_filtered  = (int32_t)(7U * IRLINE_SENSOR_SPACING_MM * 100U / 2U);
    sensor->data_ready    = 0U;
    sensor->frame_updated = 0U;
}

/*===========================================================================
 * IRLine_Task — 传感器主任务（TIMG0 ISR 5ms 槽调用）
 *
 * 完整流程：
 *   1. 遍历 8 通道
 *      a) GrayADC_SelectChannel(ch) — 切换 74HC4051 地址线
 *      b) 4 次 ADC 过采样取均值 — 抑制随机噪声
 *      c) 存入 sensor->raw_value[ch]
 *
 *   2. 固定阈值二值化
 *      raw > IRLINE_THRESHOLD → digital_bits = 1（黑线）
 *      raw ≤ IRLINE_THRESHOLD → digital_bits = 0（白线）
 *
 *   3. 构建合并字节 sensor->digital（bit0=S0 ... bit7=S7）
 *
 * 耗时：约 40µs @80MHz（8 × 4 × ADC 采样），ISR 安全。
 * 阈值 IRLINE_THRESHOLD 在 IR_Line.h 中配置，不会覆盖模块按键校准结果。
 *===========================================================================*/
void IRLine_Task(IRLine_Sensor_t *sensor)
{
    uint8_t  ch, s;
    uint32_t sum;
    uint8_t  i;

    if (sensor == 0) { return; }

    /* ── 第 1 步：采集 8 路原始 ADC ── */
    for (ch = 0U; ch < 8U; ch++)
    {
        GrayADC_SelectChannel(ch);          /* 74HC4051 地址线切换 */
        sum = 0UL;
        for (s = 0U; s < SAMPLES; s++)
        {
            sum += (uint32_t)API_ADC_GetValue(ADC_INST, ADC_CH);
        }
        sensor->raw_value[ch] = (uint16_t)(sum / SAMPLES);
    }

    /* ── 第 2 步：固定阈值二值化 ── */
    for (i = 0U; i < 8U; i++)
    {
        /*
         * 注意：模块原始逻辑为"越黑 → ADC 值越小（红外反射弱）"。
         * 但幻尔模块可能做了信号调理，实际极性以实测为准。
         * 如果二值化结果反了（黑白颠倒），修改 IRLINE_THRESHOLD
         * 的比较方向或阈值大小即可。
         */
        sensor->digital_bits[i] = (sensor->raw_value[i] > IRLINE_THRESHOLD) ? 1U : 0U;
    }

    /* ── 第 3 步：构建合并字节 ── */
    sensor->digital = 0U;
    for (i = 0U; i < 8U; i++)
    {
        if (sensor->digital_bits[i] != 0U)
        {
            sensor->digital |= (uint8_t)(1U << i);
        }
    }

    sensor->data_ready    = 1U;
    sensor->frame_updated = 1U;
}

/*===========================================================================
 * IRLine_PrintBits — 打印 8 路二值化 bits 到指定串口
 *
 * 输出格式：D:00111100
 *   1 = 黑线（探到），0 = 白线（未探到）
 *
 * 调用示例：
 *   IRLine_PrintBits(&g_irLine, USART1);  // 输出到板载串口
 *===========================================================================*/
void IRLine_PrintBits(const IRLine_Sensor_t *sensor, void *usart)
{
    if (sensor == 0) { return; }

    usart_printf((USART_TypeDef *)usart,
        "D:%d%d%d%d%d%d%d%d\r\n",
        sensor->digital_bits[0], sensor->digital_bits[1],
        sensor->digital_bits[2], sensor->digital_bits[3],
        sensor->digital_bits[4], sensor->digital_bits[5],
        sensor->digital_bits[6], sensor->digital_bits[7]);
}

/*===========================================================================
 * IRLine_LinePosition — 计算黑线加权中心位置 + EMA 低通滤波
 *
 * 传感器物理排列（间距 12mm，8 路从左到右）：
 *   [S0]  [S1]  [S2]  [S3]  [S4]  [S5]  [S6]  [S7]
 *     0   1200  2400  3600  4800  6000  7200  8400  ← mm × 100
 *
 * 加权公式：
 *   pos = Σ( digital_bits[i] × i × spacing × 100 ) / Σ( digital_bits[i] )
 *
 * 其中 digital_bits[i] = 1（黑线）→ 权重 = 1
 *      digital_bits[i] = 0（白线）→ 权重 = 0
 *
 * EMA 低通滤波：
 *   filtered = filtered × (1 - 1/N) + raw × (1/N)
 *
 * 丢线保护：
 *   全白（total == 0）→ 返回上一次有效位置，车不会乱转。
 *
 * 返回值：
 *   [0, 7×spacing×100] — 黑线加权中心位置（单位 0.01mm）
 *   -1                 — sensor 无效 / 从未收到数据
 *===========================================================================*/
int32_t IRLine_LinePosition(IRLine_Sensor_t *sensor)
{
    int32_t        weighted   = 0;
    int32_t        total      = 0;
    int32_t        rawPos;
    const int32_t  step       = (int32_t)(IRLINE_SENSOR_SPACING_MM * 100UL);
    const int32_t  maxPos     = 7 * step;      /* 最右位置 */
    const int32_t  centerPos  = maxPos / 2;    /* 居中位置 */
    uint8_t        i;

    if ((sensor == 0) || (sensor->data_ready == 0U))
    {
        return -1;  /* 传感器未就绪 */
    }

    /* 首次调用 → 初始化为居中 */
    if (sensor->pos_filtered < 0)
    {
        sensor->pos_filtered = centerPos;
    }

    /* 加权累加：黑线探头参与计算 */
    for (i = 0U; i < 8U; i++)
    {
        if (sensor->digital_bits[i] != 0U)   /* 1 = 黑线 */
        {
            weighted += (int32_t)i * step;
            total    += 1;
        }
    }

    /* 全白 / 丢线 → 保持上一次有效位置 */
    if (total == 0)
    {
        return sensor->pos_filtered;
    }

    rawPos = weighted / total;

    /* 限幅到有效范围 */
    if (rawPos < 0)      { rawPos = 0; }
    if (rawPos > maxPos) { rawPos = maxPos; }

#if IRLINE_POSITION_SMOOTHING > 0U
    /* EMA 低通：filtered = filtered × (1 - 1/N) + raw × (1/N) */
    sensor->pos_filtered = sensor->pos_filtered
                 - sensor->pos_filtered / (int32_t)(IRLINE_POSITION_SMOOTHING)
                 + rawPos                / (int32_t)(IRLINE_POSITION_SMOOTHING);
#else
    sensor->pos_filtered = rawPos;
#endif

    return sensor->pos_filtered;
}
