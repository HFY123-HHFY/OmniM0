/*
 * IR_Line.c — 幻尔 八路红外巡线模块 UART 驱动
 *
 * 硬件背景：
 *   - 通信方式：UART（API_USART4, 115200 bps）
 *   - 数字电平模式：模块收到 0x01 后持续发送单字节（每 bit = 1 个探头）
 *   - 模拟值模式：  模块收到 0x02 后持续发送帧格式（0x55 0xAA ...）
 *
 * 架构约定（遵循 OmniM0 分层规范）：
 *   - 串口资源由 Enroll 注册层统一管理（API_USART4）
 *   - ISR 上半部：IRLine_RxPush() — 环形缓冲入队（< 1µs）
 *   - ISR 下半部：IRLine_Task() — TIMG0 5ms 槽解析（< 5µs）
 *   - BSP 层不直接写寄存器，通过 API_USART 操作
 */

#include "IR_Line.h"
#include "usart.h"              /* USART4 宏定义         */
#include "My_Usart/My_Usart.h"  /* usart_printf / USART1~4 */
#include "ti/driverlib/dl_uart_main.h"  /* DL_UART_transmitData（非阻塞 TX） */

/*===========================================================================
 * 全局传感器实例
 *===========================================================================*/
IRLine_Sensor_t g_irLine;

/*===========================================================================
 * 环形缓冲区（中断上半部/下半部共享）
 *===========================================================================*/
#define IRLINE_RX_BUF_SIZE  64U    /* 数字模式每秒仅 ~20-100 字节，64 足够 */

static uint8_t           s_rx_buf[IRLINE_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;  /* ISR 写索引 */
static uint16_t          s_rx_tail = 0U;  /* Task 读索引 */

/*===========================================================================
 * IRLine_RxPush — ISR 调用：仅入队（中断上半部）
 *===========================================================================*/

void IRLine_RxPush(uint8_t data)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % IRLINE_RX_BUF_SIZE);

    s_rx_buf[s_rx_head] = data;
    s_rx_head            = next;

    /* 缓冲满 → 丢弃最老 1 字节（滑动窗口，防止死锁） */
    if (next == s_rx_tail)
    {
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % IRLINE_RX_BUF_SIZE);
    }
}

/*===========================================================================
 * IRLine_Task — TIMG0 ISR 5ms 调用：解析最新字节（中断下半部）
 *
 * 数字电平模式下，模块持续发送单字节。
 * 每 5ms 取缓冲中最新的字节作为当前传感器状态。
 * 如果无新数据，保留上一拍状态。
 *===========================================================================*/

void IRLine_Task(IRLine_Sensor_t *sensor)
{
    uint8_t  latest;
    uint8_t  has_new;
    uint8_t  i;

    if (sensor == 0)
    {
        return;
    }

    /* ── 消费环形缓冲，保留最后一个字节 ── */
    has_new = 0U;
    latest  = 0U;

    while (s_rx_tail != s_rx_head)
    {
        latest   = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % IRLINE_RX_BUF_SIZE);
        has_new  = 1U;
    }

    if (has_new == 0U)
    {
        sensor->frame_updated = 0U;
        return;
    }

    /* ── 解析单字节为 8 路数字状态 ── */
    for (i = 0U; i < 8U; i++)
    {
#if IRLINE_INVERT_DIGITAL
        /*
         * 模块原始：bit=1 → 黑线
         * 取反后：  bit=0 → 黑线（在线），符合项目惯例
         */
        sensor->digital_bits[i] = ((latest >> i) & 0x01U) ? 0U : 1U;
#else
        sensor->digital_bits[i] = (uint8_t)((latest >> i) & 0x01U);
#endif
    }

    /* 构建合并字节：bit0=S0 ... bit7=S7 */
    sensor->digital = 0U;
    for (i = 0U; i < 8U; i++)
    {
        if (sensor->digital_bits[i] == 0U)
        {
            sensor->digital |= (uint8_t)(1U << i);
        }
    }

    sensor->data_ready    = 1U;
    sensor->frame_updated = 1U;
}

/*===========================================================================
 * IRLine_Init — 初始化：发送模式指令，模块开始自动上报
 *===========================================================================*/

void IRLine_Init(IRLine_Sensor_t *sensor)
{
    uint8_t  i;
    uint16_t j;

    if (sensor == 0)
    {
        return;
    }

    /* ── 结构体清零 ── */
    for (i = 0U; i < 8U; i++)
    {
        sensor->digital_bits[i] = 1U;  /* 默认全白（离线），安全初始值 */
    }
    sensor->digital       = 0xFFU;
    sensor->pos_filtered  = (int32_t)(7U * IRLINE_SENSOR_SPACING_MM * 100U / 2U);
    sensor->data_ready    = 0U;
    sensor->frame_updated = 0U;

    /* ── 清空环形缓冲 ── */
    s_rx_head = 0U;
    s_rx_tail = 0U;
    for (j = 0U; j < IRLINE_RX_BUF_SIZE; j++)
    {
        s_rx_buf[j] = 0U;
    }

    /*
     * 发送模式指令，模块进入数字电平自动上报模式。
     *
     * 直接写 USART4 TXDATA，不经过 TI DriverLib 的 BUSY 死等。
     * （DL_UART_transmitDataBlocking 会在写完数据后 while(BUSY)，
     *   如果 USART4 引脚/时钟有问题，BUSY 永远不清零 → 卡死。）
     */
    while (DL_UART_isTXFIFOFull(USART4)) {}   /* 等 FIFO 有空位（正常立刻过） */
    DL_UART_transmitData(USART4, IRLINE_INIT_CMD);
    /* 不等 BUSY */
}

/*===========================================================================
 * 调试打印
 *===========================================================================*/

/*
 * 打印 8 路二值化 bits（纯 0/1）。
 * 输出格式：D:00111100
 *
 * 解读：0=黑（在线上），1=白（离线）
 *   例 00111100 → 中间 4 路（S2~S5）看到黑线，两侧看到白
 */
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
 * 巡线位置计算（供 PID 巡线控制使用）
 *===========================================================================*/

/*
 * 计算黑线位置 — 加权平均法 + EMA 低通滤波。
 *
 * 原理：
 *   传感器排列（间距 12mm，8 路从左到右）：
 *    [S0] [S1] [S2] [S3] [S4] [S5] [S6] [S7]
 *     0   1200  2400  3600  4800  6000  7200  8400  ← mm × 100
 *
 *   黑线处 digital_bits[i] = 0 → 权重 = 1
 *   白线处 digital_bits[i] = 1 → 权重 = 0
 *
 *   加权公式：
 *     pos = Σ( weight[i] * i * spacing * 100 ) / Σ( weight[i] )
 *
 *   再经 EMA 低通：
 *     filtered = filtered * (1 - 1/N) + pos * (1/N)
 *
 *   丢线保护：全白时保持上一次有效位置。
 */
int32_t IRLine_LinePosition(IRLine_Sensor_t *sensor)
{
    int32_t        weighted   = 0;
    int32_t        total      = 0;
    int32_t        rawPos;
    const int32_t  step       = (int32_t)(IRLINE_SENSOR_SPACING_MM * 100UL);
    const int32_t  maxPos     = 7 * step;
    const int32_t  centerPos  = maxPos / 2;
    uint8_t        i;

    if ((sensor == 0) || (sensor->data_ready == 0U))
    {
        return -1;
    }

    /* 首次调用 → 初始化为居中 */
    if (sensor->pos_filtered < 0)
    {
        sensor->pos_filtered = centerPos;
    }

    for (i = 0U; i < 8U; i++)
    {
        if (sensor->digital_bits[i] == 0U)   /* 0 = 黑线（在线） */
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
