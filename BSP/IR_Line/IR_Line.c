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
#define SAMPLES    8U

/*===========================================================================
 * 全局传感器实例
 *===========================================================================*/
IRLine_Sensor_t g_irLine;

/*===========================================================================
 * IRLine_Init — 初始化传感器结构体
 *
 * 仅清零结构体字段 + 设置默认 EMA 滤波中心位置。
 * GPIO 三根地址线（AD0/AD1/AD2）由 GrayADC_Init 已配置为推挽输出，
 * 本函数不重复操作硬件。
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
        sensor->digital_bits[i] = 0U;
        sensor->min_adc[i]      = 4095U; /* 初始化为 ADC 满量程，运行中自动收敛到白底 */
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
 *   1. 遍历 8 通道，GrayADC_SelectChannel 选通，4 次过采样取均值
 *   2. 固定阈值二值化：raw > IRLINE_THRESHOLD → 黑线(1)
 *   3. 构建合并字节
 *
 * 耗时：约 40µs @80MHz（8 × 4 × ADC 采样），ISR 安全。
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

    /* ── 白底基线跟踪：逐通道运行最小值 ── */
    /*
     * 幻尔模块：黑线吸收红外 → ADC 升高。白底 ADC 最低。
     * 运行最小值自然收敛到白底基线。
     * 初始值 4095，几帧内自动收敛。收敛后：
     *   min_adc[i] ≈ 白底 ADC ≈ 100~300
     *   黑线处 raw > min_adc，差值参与加权；白底处 raw ≈ min_adc，差值≈0，不干扰位置计算。
     */
    for (i = 0U; i < 8U; i++)
    {
        if (sensor->raw_value[i] < sensor->min_adc[i])
        {
            sensor->min_adc[i] = sensor->raw_value[i];
        }
    }

    /* ── 第 2 步：固定阈值二值化 ── */
    for (i = 0U; i < 8U; i++)
    {
        /*
         * 模块原始逻辑：红外反射弱（黑线吸收）→ ADC 值升高。
         * raw > threshold → 判为黑线。
         * 如果实测极性反了，修改 IRLINE_THRESHOLD 比较方向即可。
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
 * IRLine_PrintRaw — 打印 8 路原始 ADC 值（校准诊断用）
 *
 * 输出示例：RAW:150 180 2100 2050 1980 170 160 140
 * 放白纸上 8 路应接近一致；放黑线上，黑线位置的几路应明显偏高。
 * 若白纸上 8 路差异大 → 按模块校准按键重新校准。
 *===========================================================================*/
void IRLine_PrintRaw(const IRLine_Sensor_t *sensor, void *usart)
{
    if (sensor == 0) { return; }
    usart_printf((USART_TypeDef *)usart,
        "RAW:%d %d %d %d %d %d %d %d\r\n",
        sensor->raw_value[0], sensor->raw_value[1],
        sensor->raw_value[2], sensor->raw_value[3],
        sensor->raw_value[4], sensor->raw_value[5],
        sensor->raw_value[6], sensor->raw_value[7]);
}

/*===========================================================================
 * IRLine_PrintBits — 打印 8 路二值化 bits
 *
 * 输出格式：D:00111100
 *   1 = 黑线（探到），0 = 白线（未探到）
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
 * IRLine_PrintLinePos — 打印线位置 + 偏差 + 二值化（PID 调参专用）
 *
 * 输出示例（12mm 间距）：POS:4200 E:-120 D:00111100
 *   POS = 黑线加权位置 (0~8400，单位 0.01mm)
 *   E   = 偏差 (POS - 中心 4200)，负=偏左，正=偏右
 *   D   = 8 路二值化状态（1=黑，0=白）
 *===========================================================================*/
void IRLine_PrintLinePos(IRLine_Sensor_t *sensor, void *usart)
{
    if (sensor == 0) { return; }

    int32_t pos    = IRLine_LinePosition(sensor);
    int32_t center = (int32_t)(7U * IRLINE_SENSOR_SPACING_MM * 100U / 2U);
    int32_t error  = pos - center;

    usart_printf((USART_TypeDef *)usart,
        "POS:%d E:%d D:%d%d%d%d%d%d%d%d\r\n",
        pos, error,
        sensor->digital_bits[0], sensor->digital_bits[1],
        sensor->digital_bits[2], sensor->digital_bits[3],
        sensor->digital_bits[4], sensor->digital_bits[5],
        sensor->digital_bits[6], sensor->digital_bits[7]);
}

/*===========================================================================
 * IRLine_LinePosition — 连续灰度加权中心位置 + EMA 低通滤波
 *
 * 传感器物理排列（间距 12mm，8 路从左到右）：
 *   [S0]  [S1]  [S2]  [S3]  [S4]  [S5]  [S6]  [S7]
 *     0   1200  2400  3600  4800  6000  7200  8400  ← mm × 100
 *
 * 权重 = raw_value[i]（连续 ADC 值），越黑 → ADC 值越高 → 权重越大。
 * 与 GrayADC 的 dark = bits - normalized 同理，
 * raw_value 本身就是连续量，位置计算平滑无离散跳变。
 *
 * EMA 低通滤波：
 *   filtered = filtered × (1 - 1/N) + raw × (1/N)
 *
 * 丢线保护：
 *   全白（total < 50）→ 返回上一次有效位置，车不会乱转。
 *
 * 返回值：
 *   [0, 7×spacing×100] — 黑线加权中心位置（单位 0.01mm）
 *   -1                 — sensor 无效 / 从未收到数据
 *===========================================================================*/
int32_t IRLine_LinePosition(IRLine_Sensor_t *sensor)
{
    int32_t        weighted   = 0;
    int32_t        total      = 0;
    int32_t        dark;
    int32_t        rawPos;
    const int32_t  step       = (int32_t)(IRLINE_SENSOR_SPACING_MM * 100UL);
    const int32_t  maxPos     = 7 * step;
    const int32_t  centerPos  = maxPos / 2;
    uint8_t        i;

    if ((sensor == 0) || (sensor->data_ready == 0U))
    {
        return -1;
    }

    if (sensor->pos_filtered < 0)
    {
        sensor->pos_filtered = centerPos;
    }

    /*
     * 连续灰度加权：dark = raw_value[i] - min_adc[i]。
     *
     * 扣除白底基线后：
     *   - 白底通道 → raw ≈ min_adc → dark ≈ 0 → 不影响位置计算
     *   - 黑线通道 → raw ≫ min_adc → dark > 0 → 按黑度加权
     *
     * 这与 GrayADC 的 dark = bits - normalized 等价：
     *   两者都让白底权重归零，只有黑线处有有效权重。
     *   白底 ADC 噪声不再进入加权，位置输出干净。
     */
    for (i = 0U; i < 8U; i++)
    {
        dark = (int32_t)sensor->raw_value[i] - (int32_t)sensor->min_adc[i];
        if (dark < 0) { dark = 0; }

        weighted += dark * (int32_t)i * step;
        total    += dark;
    }

    /* 全白 / 丢线 → 保持上一次有效位置 */
    if (total < 50)
    {
        return sensor->pos_filtered;
    }

    rawPos = weighted / total;

    if (rawPos < 0)      { rawPos = 0; }
    if (rawPos > maxPos) { rawPos = maxPos; }

#if IRLINE_POSITION_SMOOTHING > 0U
    sensor->pos_filtered = sensor->pos_filtered
                 - sensor->pos_filtered / (int32_t)(IRLINE_POSITION_SMOOTHING)
                 + rawPos                / (int32_t)(IRLINE_POSITION_SMOOTHING);
#else
    sensor->pos_filtered = rawPos;
#endif

    return sensor->pos_filtered;
}
