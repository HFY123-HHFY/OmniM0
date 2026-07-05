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
 *   - 驱动内部所有细节收敛在本文件，不污染 main.c
 */

#include "gray_adc.h"
#include "gpio.h"     /* API_GPIO_InitOutput / API_GPIO_Write */
#include "adc.h"      /* API_ADC_GetValue */
#include "Delay.h"    /* Delay_us */

/*===========================================================================
 * 默认校准值（来自 gray_adc.h 宏定义）
 *===========================================================================*/
static const uint16_t s_defaultWhite[8] = GRAY_ADC_WHITE_DEFAULT;
static const uint16_t s_defaultBlack[8] = GRAY_ADC_BLACK_DEFAULT;

/*===========================================================================
 * ADC 实例与通道 — 与 G3507_hw_config.h 中的 HW_ADC_MAP 保持一致
 *===========================================================================*/
#define GRAY_ADC_INST    API_ADC1       /* ADC 外设实例              */
#define GRAY_ADC_CH      API_ADC_CH6    /* ADC 通道 (PB20)           */

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
        /* 步骤 1：选通当前通道 */
        GrayADC_SelectChannel(ch);

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
            /* 高于白阈值 → 位 置 1 */
            sensor->digital |= (uint8_t)(1U << i);
        }
        else if (sensor->raw_value[i] < sensor->threshold_black[i])
        {
            /* 低于黑阈值 → 位 清 0 */
            sensor->digital &= (uint8_t)(~(1U << i));
        }
        /* else: 中间灰度 → 保持 bit(i) 不变（迟滞） */
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
 * 校准模式 (GRAY_ADC_CALIBRATION_MODE == 1)：
 *   - 仅采集原始 ADC 值到 raw_value[]
 *   - 不做二值化和归一化
 *   - 用户通过串口读取 raw_value 来观察数据、确定 calib_white/calib_black
 *
 * 正常模式 (GRAY_ADC_CALIBRATION_MODE == 0)：
 *   - 完整流程：采集 → 二值化 → 归一化
 *   - 如果尚未调用 GrayADC_InitSensor()，自动使用默认校准值初始化一次
 */
void GrayADC_Task(GrayADC_Sensor_t *sensor)
{
    if (sensor == 0)
    {
        return;
    }

    /* ── 第 1 步：采集 8 路原始 ADC ── */
    GrayADC_ReadAllRaw(sensor);

#if GRAY_ADC_CALIBRATION_MODE == 1U
    /*
     * 校准模式：仅采集原始值。
     * 用户可通过 sensor->raw_value 观察并确定校准参数。
     * 在 main.c 中用 usart_printf 打印 raw_value 即可：
     *
     *   usart_printf(USART1, "R:%d %d %d %d %d %d %d %d\r\n",
     *       sensor.raw_value[0], sensor.raw_value[1], ...);
     */
#else
    /*
     * 正常模式：二值化 + 归一化
     */

    /* 首次进入且未手动校准 → 使用宏定义的默认校准值 */
    if (sensor->calib_ready == 0U)
    {
        GrayADC_InitSensor(sensor, s_defaultWhite, s_defaultBlack);
    }

    /* ── 第 2 步：二值化 ── */
    GrayADC_ConvertToDigital(sensor);

    /* ── 第 3 步：归一化 ── */
    GrayADC_Normalize(sensor);
#endif
}

/*===========================================================================
 * 用户接口
 *===========================================================================*/

/*
 * 获取二值化结果。
 *
 * 返回值：
 *   8-bit 无符号数，bit0 = 第 1 路传感器，...，bit7 = 第 8 路。
 *   1 = 白色/亮（高于 white 阈值），0 = 黑色/暗（低于 black 阈值）。
 *
 * 典型用法（循线判断）：
 *   uint8_t d = GrayADC_GetDigital(&sensor);
 *   if (d == 0x00) { ... }  // 全部在线上（全黑）
 *   if (d == 0xFF) { ... }  // 全部离线（全白）
 *   if (d & 0x18)  { ... }  // 第4、5路检测到白线
 */
uint8_t GrayADC_GetDigital(const GrayADC_Sensor_t *sensor)
{
    if (sensor == 0)
    {
        return 0U;
    }
    return sensor->digital;
}

/*
 * 获取归一化结果数组指针。
 *
 * 返回值：
 *   指向 8 个 uint16_t 的指针，范围 [0, bits]。
 *   在正常模式且校准完成后有效；否则返回 NULL。
 *
 * 典型用法（循线加权位置计算）：
 *   const uint16_t *nv = GrayADC_GetNormalized(&sensor);
 *   if (nv != NULL) {
 *       int32_t weighted_sum = 0, total = 0;
 *       for (int i = 0; i < 8; i++) {
 *           weighted_sum += (int32_t)nv[i] * i * 1000;
 *           total += (int32_t)nv[i];
 *       }
 *       int32_t line_pos = (total > 0) ? (weighted_sum / total) : 0;
 *       // line_pos: 0=最左, 7000=最右
 *   }
 */
const uint16_t *GrayADC_GetNormalized(const GrayADC_Sensor_t *sensor)
{
    if ((sensor == 0) || (sensor->calib_ready == 0U))
    {
        return 0;
    }
    return sensor->normalized;
}
