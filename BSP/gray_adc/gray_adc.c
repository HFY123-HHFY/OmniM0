/*
 * gray_adc.c — 感为无MCU八路灰度传感器驱动
 *
 * 硬件背景：
 *   - 传感器基于 74HC4051 模拟开关 + 8 路红外对管
 *   - AD0/AD1/AD2 三根地址线选择通道
 *   - OUT 引脚输出模拟电压 → 单片机 ADC 采集
 *
 * 架构约定（遵循 OmniM0 分层规范）：
 *   - 引脚映射集中在 Enroll/G3507_hw_config.h
 *   - 注册→初始化 两阶段模式（Register → Init）
 *   - BSP 层不直接写寄存器，通过 API_GPIO / API_ADC 操作
 */

#include "gray_adc.h"
#include "gpio.h"     /* API_GPIO_InitOutput / API_GPIO_Write */
#include "adc.h"      /* API_ADC_GetValue */
#include "Delay.h"    /* Delay_us */
#include "My_Usart/My_Usart.h"  /* usart_printf / USART1/USART2 宏 */
#include "Control/Control.h"    /* g_graySensor */
#include <stdio.h>    /* sprintf */

/*===========================================================================
 * 默认校准值（来自 gray_adc.h 宏定义）
 *===========================================================================*/
static const uint16_t s_defaultWhite[8] = GRAY_ADC_WHITE_DEFAULT;
static const uint16_t s_defaultBlack[8] = GRAY_ADC_BLACK_DEFAULT;

/*===========================================================================
 * ADC 实例与通道 — 与 G3507_hw_config.h 中的 HW_ADC_MAP 保持一致
 *===========================================================================*/
#define GRAY_ADC_INST    API_ADC1       /* ADC 外设实例（与 HW_ADC_MAP 一致） */
#define GRAY_ADC_CH      API_ADC_CH4    /* ADC 通道（与 HW_ADC_MAP 一致，当前 CH4=B25） */

/*===========================================================================
 * 采样参数
 *===========================================================================*/
#define GRAY_ADC_SAMPLES_PER_CH   8U    /* 每通道过采样次数（均值滤波）   */
#define GRAY_ADC_SWITCH_DELAY_US  1U    /* 74HC4051 通道切换稳定延时 (us) */

/*===========================================================================
 * 配置表（由 Enroll 注册层填充）
 *===========================================================================*/
static const GrayADC_Config_t *s_configTable = 0;
static uint8_t                  s_configCount = 0U;

/*===========================================================================
 * 内部辅助宏 — 通过统一 API 写 GPIO
 *===========================================================================*/
#define GRAY_ADC_WRITE(port, pin, level) \
    API_GPIO_Write((port), (pin), (uint8_t)((level) ? 1U : 0U))

/*===========================================================================
 * 注册与初始化（Register → Init 两阶段）
 *===========================================================================*/

/*
 * 注册 GrayADC 配置表。
 * 由 Enroll_GrayADC_Register() 在启动阶段调用。
 */
void GrayADC_Register(const GrayADC_Config_t *configTable, uint8_t count)
{
    s_configTable = configTable;
    s_configCount = count;
}

/*
 * 获取当前生效的配置项。
 * 未注册时返回 NULL，后续所有操作会被安全跳过。
 */
static const GrayADC_Config_t *GrayADC_GetConfig(void)
{
    if ((s_configTable == 0) || (s_configCount == 0U))
    {
        return 0;
    }
    return &s_configTable[0];
}

/*
 * 初始化地址选择引脚。
 *
 * 操作：
 *   1. 将 AD0(PB17)、AD1(PB18)、AD2(PB19) 配置为推挽输出
 *   2. 默认选中通道 0（为安全起见）
 *
 * 前置条件：GrayADC_Register() 已被调用。
 * ADC 输入引脚 (PB20) 的初始化由 API_ADC_Init() 统一负责，此处不重复操作。
 */
void GrayADC_Init(void)
{
    const GrayADC_Config_t *cfg = GrayADC_GetConfig();
    if (cfg == 0)
    {
        return;
    }

    /* 初始化三根地址线为推挽输出（无上下拉，默认低电平） */
    API_GPIO_InitOutput(cfg->ad0Port, cfg->ad0Pin);
    API_GPIO_InitOutput(cfg->ad1Port, cfg->ad1Pin);
    API_GPIO_InitOutput(cfg->ad2Port, cfg->ad2Pin);

    /* 上电默认选通通道 0，避免地址线浮空导致的随机读数 */
    GrayADC_SelectChannel(0U);

    /* digital_bits 初始化为全白(1)，防止 ISR 在首次 ADC 采集前误判全黑 */
    {
        uint8_t _i;
        for (_i = 0U; _i < 8U; ++_i) { g_graySensor.digital_bits[_i] = 1U; }
    }
}

/*===========================================================================
 * 通道选择（74HC4051 模拟开关控制）
 *===========================================================================*/

/*
 * 选通传感器通道。
 *
 * 74HC4051 真值表（当 INH = 0 时）：
 *   ┌─────┬─────┬─────┬─────────┐
 *   │ AD2 │ AD1 │ AD0 │ 接通通道 │
 *   ├─────┼─────┼─────┼─────────┤
 *   │  0  │  0  │  0  │  Y0(第1) │
 *   │  0  │  0  │  1  │  Y1(第2) │
 *   │  0  │  1  │  0  │  Y2(第3) │
 *   │  0  │  1  │  1  │  Y3(第4) │
 *   │  1  │  0  │  0  │  Y4(第5) │
 *   │  1  │  0  │  1  │  Y5(第6) │
 *   │  1  │  1  │  0  │  Y6(第7) │
 *   │  1  │  1  │  1  │  Y7(第8) │
 *   └─────┴─────┴─────┴─────────┘
 *
 * 注意：地址线使用取反逻辑（!(channel & bit)）。
 * 这是因为参考驱动中 74HC4051 的地址引脚通过反相器连接，
 * 当单片机输出低电平时实际到达芯片为高电平。
 * 如果你的硬件没有反相，去掉 `!` 即可。
 *
 * 参数：
 *   channel — 0~7（超出范围自动 clamp 到 0~7）
 */
void GrayADC_SelectChannel(uint8_t channel)
{
    const GrayADC_Config_t *cfg = GrayADC_GetConfig();
    if (cfg == 0)
    {
        return;
    }

    /* 通道号限制在 0~7 */
    channel &= 0x07U;

    /*
     * 设置三根地址线。
     * 取反逻辑 `!` 是为了适配参考驱动的硬件设计。
     * 如果实际方向反了，去掉 `!` 或统一改为不取反。
     */
    GRAY_ADC_WRITE(cfg->ad0Port, cfg->ad0Pin, !(channel & 0x01U)); /* AD0 */
    GRAY_ADC_WRITE(cfg->ad1Port, cfg->ad1Pin, !(channel & 0x02U)); /* AD1 */
    GRAY_ADC_WRITE(cfg->ad2Port, cfg->ad2Pin, !(channel & 0x04U)); /* AD2 */

    /*
     * 等待 74HC4051 模拟开关输出稳定。
     * 根据数据手册，典型切换时间 < 500ns，1us 留有裕量。
     */
    Delay_us(GRAY_ADC_SWITCH_DELAY_US);
}

/*===========================================================================
 * ADC 数据采集
 *===========================================================================*/

/*
 * 读取全部 8 路传感器的原始 ADC 值。
 *
 * 采集流程：
 *   对每一路 (0~7)：
 *     1. 选通该通道（切换 74HC4051 地址线）
 *     2. 连续采样 8 次 ADC → 取均值（滤除电源噪声和 ADC 随机误差）
 *     3. 存入 sensor->raw_value[i]
 *
 * 结果：
 *   sensor->raw_value[0..7] 被更新为当前时刻的 8 路 ADC 均值。
 */
void GrayADC_ReadAllRaw(GrayADC_Sensor_t *sensor)
{
    uint8_t  ch;
    uint8_t  sample;
    uint32_t sum;

    if (sensor == 0)
    {
        return;
    }

    for (ch = 0U; ch < 8U; ch++)
    {
        /* 步骤 1：选通当前通道（7-ch 翻转物理顺序） */
        GrayADC_SelectChannel(7U - ch);

        /* 步骤 2：多次采样取均值（8x 过采样 + 均值滤波） */
        sum = 0UL;
        for (sample = 0U; sample < GRAY_ADC_SAMPLES_PER_CH; sample++)
        {
            sum += (uint32_t)API_ADC_GetValue(GRAY_ADC_INST, GRAY_ADC_CH);
        }
        sensor->raw_value[ch] = (uint16_t)(sum / GRAY_ADC_SAMPLES_PER_CH);
    }
}

/*===========================================================================
 * 传感器校准初始化
 *===========================================================================*/

/*
 * 传感器校准初始化。
 *
 * 此函数用白/黑校准值初始化传感器实例，内部自动完成：
 *   1. 白/黑值合法性检查（white 必须 > black，否则交换）
 *   2. 计算二值化阈值：
 *      - threshold_white = (white*2 + black) / 3  （高于此值判为"白/亮"）
 *      - threshold_black = (white + black*2) / 3  （低于此值判为"黑/暗"）
 *      - 介于两者之间的 → 保持上一状态（回滞特性，防止边界抖动）
 *   3. 计算归一化系数：
 *      - norm_factor[i] = bits / (white[i] - black[i])
 *      - 使得 normalized[i] 映射到 [0, bits] 范围
 *   4. 设置 calib_ready = 1，标记校准完成
 *
 * 参数：
 *   sensor       — 传感器实例指针（由 App 层定义和持有）
 *   calib_white  — 8 个通道在白色/浅色表面的 ADC 均值
 *   calib_black  — 8 个通道在黑色/深色表面的 ADC 均值
 *
 * 注意：如果 white==black（差值=0），该通道被标记为无效（norm_factor=0）。
 */
void GrayADC_InitSensor(GrayADC_Sensor_t *sensor,
                        const uint16_t *calib_white,
                        const uint16_t *calib_black)
{
    uint8_t  i;
    double   diff;
    uint16_t temp;

    if ((sensor == 0) || (calib_white == 0) || (calib_black == 0))
    {
        return;
    }

    /* ── 清零所有字段，确保已知初始状态 ── */
    for (i = 0U; i < 8U; i++)
    {
        sensor->raw_value[i]       = 0U;
        sensor->normalized[i]      = 0U;
        sensor->digital_bits[i]    = 0U;
        sensor->calib_white[i]     = 0U;
        sensor->calib_black[i]     = 0U;
        sensor->threshold_white[i] = 0U;
        sensor->threshold_black[i] = 0U;
        sensor->norm_factor[i]     = 0.0;
    }
    sensor->digital     = 0U;
    sensor->calib_ready = 0U;

    /* ── ADC 量程：MSPM0G3507 的 12-bit ADC，满量程 = 4096 ── */
    sensor->bits = 4096.0;

    /* ── 逐通道计算校准参数 ── */
    for (i = 0U; i < 8U; i++)
    {
        /* 保存原始校准值 */
        sensor->calib_white[i] = calib_white[i];
        sensor->calib_black[i] = calib_black[i];

        /* 确保 white > black（如果用户搞反了，自动交换） */
        if (sensor->calib_black[i] >= sensor->calib_white[i])
        {
            temp                     = sensor->calib_white[i];
            sensor->calib_white[i]   = sensor->calib_black[i];
            sensor->calib_black[i]   = temp;
        }

        /*
         * 二值化阈值：1:2 和 2:1 分界
         *
         * 例：white=2000, black=500
         *   → threshold_white = (2000*2+500)/3 = 1500
         *   → threshold_black = (2000+500*2)/3 = 1000
         *   → raw > 1500 判白，raw < 1000 判黑
         *   → 1000~1500 之间为过渡灰度区，保持上一状态
         */
        sensor->threshold_white[i] =
            (uint16_t)(((uint32_t)sensor->calib_white[i] * 2UL +
                        (uint32_t)sensor->calib_black[i]) / 3UL);
        sensor->threshold_black[i] =
            (uint16_t)(((uint32_t)sensor->calib_white[i] +
                        (uint32_t)sensor->calib_black[i] * 2UL) / 3UL);

        /*
         * 归一化系数：将 [black, white] 映射到 [0, bits]
         *   norm_factor = bits / (white - black)
         *   normalized   = (raw - black) * norm_factor
         */
        diff = (double)sensor->calib_white[i] -
               (double)sensor->calib_black[i];
        if (diff > 0.0)
        {
            sensor->norm_factor[i] = sensor->bits / diff;
        }
        else
        {
            /* white == black：此通道无效，系数置 0 */
            sensor->norm_factor[i] = 0.0;
        }
    }

    sensor->calib_ready = 1U;
}

/*===========================================================================
 * 数据处理（内部函数）
 *===========================================================================*/

/*
 * 二值化处理：
 *   将 raw_value[0..7] 与阈值比较，生成 8-bit 数字结果。
 *
 *   规则（带迟滞回环）：
 *     raw[i] >  threshold_white[i]  →  digital bit(i) = 1  （亮/白色）
 *     raw[i] <  threshold_black[i]  →  digital bit(i) = 0  （暗/黑色）
 *     介于两者之间                    →  保持 bit(i) 不变    （迟滞区）
 *
 *   迟滞回环的意义：当传感器位于黑白边界时，避免数字输出来回跳变。
 */
static void GrayADC_ConvertToDigital(GrayADC_Sensor_t *sensor)
{
    uint8_t i;

    if (sensor == 0)
    {
        return;
    }

    for (i = 0U; i < 8U; i++)
    {
        if (sensor->raw_value[i] > sensor->threshold_white[i])
        {
            /* 高于白阈值 → 位 置 1（亮/白色） */
            sensor->digital      |= (uint8_t)(1U << i);
            sensor->digital_bits[i] = 1U;
        }
        else if (sensor->raw_value[i] < sensor->threshold_black[i])
        {
            /* 低于黑阈值 → 位 清 0（暗/黑色） */
            sensor->digital      &= (uint8_t)(~(1U << i));
            sensor->digital_bits[i] = 0U;
        }
        /* else: 中间灰度 → 保持 digital bit 和 digital_bits[i] 不变（迟滞） */
    }
}

/*
 * 归一化处理：
 *   将 raw_value 映射到 [0, bits] 范围。
 *
 *   公式：normalized[i] = clamp((raw[i] - black[i]) * norm_factor[i], 0, bits)
 *
 *   用途：归一化后 8 路数据在同一尺度下可比较，适合循线算法做加权/插值。
 *   例：bits=4096 时，全白 → ~4096，全黑 → ~0。
 */
static void GrayADC_Normalize(GrayADC_Sensor_t *sensor)
{
    uint8_t  i;
    uint16_t n;
    int32_t  diff;

    if (sensor == 0)
    {
        return;
    }

    for (i = 0U; i < 8U; i++)
    {
        /* 无效通道 → 归零 */
        if (sensor->norm_factor[i] <= 0.0)
        {
            sensor->normalized[i] = 0U;
            continue;
        }

        diff = (int32_t)sensor->raw_value[i] -
               (int32_t)sensor->calib_black[i];

        if (diff < 0)
        {
            n = 0U;
        }
        else
        {
            n = (uint16_t)((double)diff * sensor->norm_factor[i]);

            /* 限幅：不超过 ADC 量程 */
            if (n > (uint16_t)sensor->bits)
            {
                n = (uint16_t)sensor->bits;
            }
        }

        sensor->normalized[i] = n;
    }
}

/*===========================================================================
 * 传感器主任务
 *===========================================================================*/

/*
 * 传感器主任务 — 每个控制周期调用一次。
 *
 * 完整流程：采集 raw_value → 二值化 → 归一化。
 * 如果尚未调用 GrayADC_InitSensor()，自动使用默认校准值初始化一次。
 */
void GrayADC_Task(GrayADC_Sensor_t *sensor)
{
    if (sensor == 0)
    {
        return;
    }

    /* ── 第 1 步：采集 8 路原始 ADC ── */
    GrayADC_ReadAllRaw(sensor);

    /* 首次进入且未手动校准 → 使用宏定义的默认校准值 */
    if (sensor->calib_ready == 0U)
    {
        GrayADC_InitSensor(sensor, s_defaultWhite, s_defaultBlack);
    }

    /* ── 第 2 步：二值化 ── */
    GrayADC_ConvertToDigital(sensor);

    /* ── 第 3 步：归一化 ── */
    GrayADC_Normalize(sensor);
}

/*===========================================================================
 * 调试打印
 *===========================================================================*/

/*
 * 打印 8 路原始 ADC 值 — 用于校准。
 * 输出示例：RAW: 2100 2050 1980 2120 1500 2080 2030 2070
 *
 * 用法：
 *   1. 传感器放白色表面 → 记录 8 个值 → 填入 GRAY_ADC_WHITE_DEFAULT
 *   2. 传感器放黑色表面 → 记录 8 个值 → 填入 GRAY_ADC_BLACK_DEFAULT
 */
void GrayADC_PrintRaw(const GrayADC_Sensor_t *sensor, void *usart)
{
    if (sensor == 0) { return; }

    usart_printf((USART_TypeDef *)usart,
        "RAW: %d %d %d %d %d %d %d %d\r\n",
        sensor->raw_value[0], sensor->raw_value[1],
        sensor->raw_value[2], sensor->raw_value[3],
        sensor->raw_value[4], sensor->raw_value[5],
        sensor->raw_value[6], sensor->raw_value[7]);
}

/*
 * 打印 8 路二值化 bits（纯 0/1
 * 输出示例：D:00111100
 *
 * 解读：0=黑（在线上），1=白（离线）
 *   例 00111100 → 中间 4 路（S2~S5）看到黑线，两侧看到白
 */
void GrayADC_PrintBits(const GrayADC_Sensor_t *sensor, void *usart)
{
    if (sensor == 0) { return; }

    usart_printf((USART_TypeDef *)usart,
        "D:%d%d%d%d%d%d%d%d\r\n",
        sensor->digital_bits[0], sensor->digital_bits[1],
        sensor->digital_bits[2], sensor->digital_bits[3],
        sensor->digital_bits[4], sensor->digital_bits[5],
        sensor->digital_bits[6], sensor->digital_bits[7]);
}

/*
 * 打印线位置 + 偏差 + 二值化 — PID 调参专用，一屏看全。
 *
 * 输出示例（12mm 间距）：POS:4200 E:-120 D:00111100
 *   POS = 黑线加权位置 (0~8400)
 *   E   = 偏差 (POS - 中心)，负=偏左，正=偏右
 *   D   = 8 路二值化状态
 */
void GrayADC_PrintLinePos(const GrayADC_Sensor_t *sensor, void *usart)
{
    if (sensor == 0) { return; }

    int32_t pos    = GrayADC_LinePosition(sensor);
    int32_t center = (int32_t)(7U * GRAY_ADC_SENSOR_SPACING_MM * 100U / 2U);
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
 * 巡线位置计算（供 PID 巡线控制使用）
 *===========================================================================*/

/*
 * 计算黑线位置 — 加权平均法 + EMA 低通滤波。
 *
 * 原理：
 *   传感器排列（假设间距 12mm，8 路从左到右）：
 *    [S0] [S1] [S2] [S3] [S4] [S5] [S6] [S7]
 *     0   1200  2400  3600  4800  6000  7200  8400  ← mm × 100
 *
 *   深色（黑线）吸收红外光 → ADC 值小 → normalized 值小。
 *   为让黑线处权重更大，用 (bits - normalized[i]) 取反。
 *
 *   加权公式（raw）：
 *     pos = Σ( (bits - nv[i]) * i * spacing * 100 ) / Σ( bits - nv[i] )
 *
 *   再经 EMA 低通：
 *     filtered = filtered * (1 - 1/N) + pos * (1/N)
 *
 *   当分母太小（全白/丢线）时，返回上一次有效位置（static 保持）。
 *
 * 返回值：
 *   [0, 7*spacing*100] — 黑线加权中心位置（单位 = 0.01mm）
 *   -1                 — sensor 无效/未校准
 */
int32_t GrayADC_LinePosition(const GrayADC_Sensor_t *sensor)
{
    static int32_t s_filtered = -1;        /* EMA 滤波历史值 */
    int32_t        weighted   = 0;
    int32_t        total      = 0;
    int32_t        dark;
    int32_t        rawPos;
    const int32_t  step       = (int32_t)(GRAY_ADC_SENSOR_SPACING_MM * 100UL); /* 单步间距 */
    const int32_t  maxPos     = 7 * step;   /* 最右位置 = 7 × 间距 × 100 */
    const int32_t  centerPos  = maxPos / 2; /* 居中位置 */
    uint8_t i;

    if ((sensor == 0) || (sensor->calib_ready == 0U))
    {
        s_filtered = -1;
        return -1;
    }

    /* 首次调用 → 初始化为居中的滤波值 */
    if (s_filtered < 0)
    {
        s_filtered = centerPos;
    }

    for (i = 0U; i < 8U; i++)
    {
        /*
         * dark = 归一化值的取反：值越大表示越暗（越可能是黑线）。
         * 用 4096 减去归一化值，使得黑线处权重最大。
         */
        dark = (int32_t)((uint16_t)sensor->bits) - (int32_t)sensor->normalized[i];
        if (dark < 0) { dark = 0; }

        /* 物理位置 = i × 间距 × 100 */
        weighted += dark * (int32_t)i * step;
        total    += dark;
    }

    /* 全白 / 丢线 → 保持上一次有效位置，防止车乱转 */
    if (total < 50)
    {
        return s_filtered;
    }

    rawPos = weighted / total;

    /* 限幅 */
    if (rawPos < 0)       { rawPos = 0; }
    if (rawPos > maxPos)  { rawPos = maxPos; }

#if GRAY_ADC_POSITION_SMOOTHING > 0U
    /* EMA 低通：s_filtered = s_filtered*(1-1/N) + rawPos*(1/N) */
    s_filtered = s_filtered
               - s_filtered / (int32_t)(GRAY_ADC_POSITION_SMOOTHING)
               + rawPos     / (int32_t)(GRAY_ADC_POSITION_SMOOTHING);
#else
    s_filtered = rawPos;
#endif

    return s_filtered;
}
