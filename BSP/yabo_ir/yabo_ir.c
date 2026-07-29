/*
 * yabo_ir.c — 亚博智能 八路红外巡线模块 驱动
 *
 * 硬件背景：
 *   - 通信方式：UART（API_USART4, 115200 bps）
 *   - 协议格式：上位机 → 模块: $0,0,1#（仅数字量）
 *              模块 → 上位机: $D,x1:V,x2:V,...,x8:V#
 *              V=0=白(未探到), V=1=黑(探到黑线)
 *   - 初始化命令：$0,0,1#
 *
 * 架构约定（遵循 OmniM0 分层规范）：
 *   - 串口资源由 Enroll 注册层统一管理（API_USART4）
 *   - ISR 上半部：YaboIR_RxPush() — 环形缓冲入队（< 1µs）
 *   - ISR 下半部：YaboIR_Task() — TIMG0 5ms 槽解析帧（< 10µs）
 *   - BSP 层不直接写寄存器，通过 API_USART / usart_send_byte 操作
 */

#include "yabo_ir.h"
#include "usart.h"              /* API_USART_WriteByte / API_USART_Id_t */
#include "My_Usart/My_Usart.h"  /* usart_send_byte / usart_printf / USART1~4 */
#include <string.h>             /* strchr */

/*===========================================================================
 * 环形缓冲区（中断上半部/下半部共享，head 由 ISR 写）
 *===========================================================================*/
#define YABO_IR_RX_BUF_SIZE  256U

static uint8_t           s_rx_buf[YABO_IR_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;  /* ISR 写索引  */
static uint16_t          s_rx_tail = 0U;  /* Task 读索引 */

/*===========================================================================
 * 帧解析 — 内部静态缓冲
 *===========================================================================*/
#define FRAME_BUF_SIZE  64U    /* "$D,x1:0,...,x8:0#" ≈ 40 字节，64 足够 */

/*
 * 解析状态机：
 *   STATE_IDLE       — 等待 '$'
 *   STATE_DATA       — 收集帧体字节（'D' 到 '#' 之间）
 */
#define STATE_IDLE   0U
#define STATE_DATA   1U

/*===========================================================================
 * 内部辅助
 *===========================================================================*/

/*
 * 从帧体中提取 8 路传感器值。
 *
 * 帧体格式：",x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0"
 * 策略：扫描 ':' 字符，紧随其后的字符（'0' 或 '1'）即为该路数据。
 * 8 个传感器严格按顺序出现。
 *
 * 参数：
 *   body  — 帧体字符串（含结尾 '\0'）
 *   bits  — 输出数组 bits[0..7]，填写模块原始值 0 或 1
 *
 * 返回值：成功解析的个数（应为 8）
 */
static uint8_t YaboIR_ParseBody(const char *body, uint8_t *bits)
{
    uint8_t       count = 0U;
    const char   *ptr   = body;

    while (count < 8U)
    {
        /* 找下一个 ':' */
        ptr = strchr(ptr, ':');
        if ((ptr == 0) || (ptr[1] == '\0'))
        {
            break;  /* 帧体异常（短于预期），已解析的部分有效 */
        }

        /* 读取冒号后的值：'0'=48='\x30', '1'=49='\x31' */
        if (ptr[1] == '1')
        {
            bits[count] = 1U;
        }
        else
        {
            bits[count] = 0U;  /* 包括 '0' 和异常值，安全默认 0 */
        }

        count++;
        ptr += 2;  /* 跳过 ":V" */
    }

    return count;
}

/*===========================================================================
 * YaboIR_RxPush — ISR 调用：仅入队（中断上半部）
 *===========================================================================*/

void YaboIR_RxPush(uint8_t data)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % YABO_IR_RX_BUF_SIZE);

    s_rx_buf[s_rx_head] = data;
    s_rx_head            = next;

    /* 缓冲满 → 丢弃最老 1 字节（滑动窗口，防止死锁） */
    if (next == s_rx_tail)
    {
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % YABO_IR_RX_BUF_SIZE);
    }
}

/*===========================================================================
 * YaboIR_Task — TIMG0 ISR 5ms 调用：解析已缓冲的字节（中断下半部）
 *
 * 状态机：
 *   IDLE → 收到 '$' → DATA（重置帧缓冲）
 *   DATA → 收到 '#' → 解析帧 → 更新 sensor → IDLE
 *        → 其他字节 → 入帧缓冲（保护截断）
 *
 * 帧体最大约 40 字节，每 5ms 出队最多 512 字节（115200bps×5ms≈72B），
 * 极限情况下也只处理约 13 帧，耗时 < 100µs，ISR 安全。
 *===========================================================================*/

void YaboIR_Task(YaboIR_Sensor_t *sensor)
{
    static uint8_t  state      = STATE_IDLE;
    static char     frame[FRAME_BUF_SIZE];
    static uint8_t  frame_idx  = 0U;
    uint8_t data;

    if (sensor == 0)
    {
        return;
    }

    /* 每周期重置 frame_updated，有新帧时再置位 */
    sensor->frame_updated = 0U;

    while (s_rx_tail != s_rx_head)
    {
        data       = s_rx_buf[s_rx_tail];
        s_rx_tail  = (uint16_t)((s_rx_tail + 1U) % YABO_IR_RX_BUF_SIZE);

        switch (state)
        {
        case STATE_IDLE:
            if (data == (uint8_t)'$')
            {
                state     = STATE_DATA;
                frame_idx = 0U;
                /* '$' 不入帧缓冲，帧体从下一字节开始 */
            }
            break;

        case STATE_DATA:
            if (data == (uint8_t)'#')
            {
                /* ── 帧结束：终止字符串 → 解析 ── */
                state = STATE_IDLE;

                if (frame_idx < FRAME_BUF_SIZE)
                {
                    frame[frame_idx] = '\0';
                }
                else
                {
                    frame[FRAME_BUF_SIZE - 1U] = '\0';
                }

                /* 校验帧头必须为 'D' */
                if (frame[0] != 'D')
                {
                    break;  /* 非法帧，丢弃 */
                }

                /*
                 * 解析帧体：frame = "D,x1:0,x2:0,...,x8:0"
                 * 从 frame+1 开始跳过 'D'（body 以逗号开头：",x1:0,..."）
                 * YaboIR_ParseBody 从 ':' 扫描，不依赖逗号位置。
                 */
                {
                    uint8_t raw[8];
                    uint8_t i;
                    uint8_t count = YaboIR_ParseBody(frame + 1U, raw);

                    if (count == 8U)
                    {
                        /* ── 写入 sensor（原子更新，字段间无依赖）── */
#if YABO_IR_INVERT_DIGITAL
                        /* 取反：模块 1=黑 → GrayADC 惯例 0=黑 */
                        for (i = 0U; i < 8U; i++)
                        {
                            sensor->raw_bits[i]    = raw[i];
                            sensor->digital_bits[i] = (raw[i] == 1U) ? 0U : 1U;
                        }
#else
                        /* 直通：保留模块原始值 */
                        for (i = 0U; i < 8U; i++)
                        {
                            sensor->raw_bits[i]    = raw[i];
                            sensor->digital_bits[i] = raw[i];
                        }
#endif

                        /* 构建合并字节：bit0=S0 ... bit7=S7（GrayADC 惯例） */
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
                    /* else: count < 8 → 帧残缺，丢弃（保留上一帧有效数据） */
                }
            }
            else
            {
                /* ── 帧内字节：入缓冲 ── */
                if (frame_idx < (FRAME_BUF_SIZE - 1U))
                {
                    frame[frame_idx] = (char)data;
                    frame_idx++;
                }
                /* else: 帧体超长（异常），静默丢弃后续字节直到 '#' */
            }
            break;

        default:
            state = STATE_IDLE;
            break;
        }
    }
}

/*===========================================================================
 * YaboIR_Init — 初始化传感器结构体
 *===========================================================================*/

void YaboIR_Init(YaboIR_Sensor_t *sensor)
{
    uint8_t  i;
    uint16_t j;

    if (sensor == 0)
    {
        return;
    }

    for (i = 0U; i < 8U; i++)
    {
        sensor->raw_bits[i]    = 0U;
        sensor->digital_bits[i] = 1U;  /* 默认全白（离线），安全初始值 */
    }
    sensor->digital       = 0xFFU;     /* 全白 */
    sensor->pos_filtered  = (int32_t)(7U * YABO_IR_SENSOR_SPACING_MM * 100U / 2U); /* 居中 */
    sensor->data_ready    = 0U;
    sensor->frame_updated = 0U;

    /* 清空环形缓冲 */
    s_rx_head = 0U;
    s_rx_tail = 0U;
    for (j = 0U; j < YABO_IR_RX_BUF_SIZE; j++)
    {
        s_rx_buf[j] = 0U;
    }
}

/*===========================================================================
 * YaboIR_SendCmd — 向模块发送初始化命令
 *
 * 命令格式：$0,0,1#
 *   字段 1 = 0  → 不进入校准模式
 *   字段 2 = 0  → 不发送模拟量数据
 *   字段 3 = 1  → 发送数字量数据
 *
 * 调用时机：USART4 已初始化后，TIMG0 启动前。
 * 发送后模块立刻开始持续上报 "$D,x1:0,...#" 帧。
 *
 * 使用 API_USART_WriteByte 直接字节发送（阻塞 TX FIFO 模式），
 * 不在 ISR 上下文，无异步竞争。
 *===========================================================================*/

void YaboIR_SendCmd(void)
{
    const char *cmd = "$0,0,1#";
    uint8_t     i;

    for (i = 0U; cmd[i] != '\0'; i++)
    {
        API_USART_WriteByte(API_USART4, (uint8_t)cmd[i]);
    }
}

/*===========================================================================
 * 调试打印
 *===========================================================================*/

/*
 * 打印 8 路二值化 bits（纯 0/1）。
 * 输出示例：D:00111100
 *
 * 解读：0=黑（在线上），1=白（离线）
 *   例 00111100 → 中间 4 路（S2~S5）看到黑线，两侧看到白
 */
void YaboIR_PrintBits(const YaboIR_Sensor_t *sensor, void *usart)
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
void YaboIR_PrintLinePos(YaboIR_Sensor_t *sensor, void *usart)
{
    if (sensor == 0) { return; }

    int32_t pos    = YaboIR_LinePosition(sensor);
    int32_t center = (int32_t)(7U * YABO_IR_SENSOR_SPACING_MM * 100U / 2U);
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
 *   黑线处 digital_bits[i] = 0 → 权重 = 1（参与位置计算）
 *   白线处 digital_bits[i] = 1 → 权重 = 0（忽略）
 *
 *   加权公式：
 *     pos = Σ( weight[i] * i * spacing * 100 ) / Σ( weight[i] )
 *
 *   再经 EMA 低通：
 *     filtered = filtered * (1 - 1/N) + pos * (1/N)
 *
 *   丢线保护：全白（无探头看到黑线）时保持上一次有效位置。
 *
 * 返回值：
 *   [0, 7*spacing*100] — 黑线加权中心位置（单位 = 0.01mm）
 *   -1                 — sensor 无效/从未收到数据
 */
int32_t YaboIR_LinePosition(YaboIR_Sensor_t *sensor)
{
    int32_t        weighted  = 0;
    int32_t        total     = 0;
    int32_t        rawPos;
    const int32_t  step      = (int32_t)(YABO_IR_SENSOR_SPACING_MM * 100UL);
    const int32_t  maxPos    = 7 * step;   /* 最右位置 = 7 × 间距 × 100 */
    const int32_t  centerPos = maxPos / 2; /* 居中位置 */
    uint8_t        i;

    if ((sensor == 0) || (sensor->data_ready == 0U))
    {
        return -1;
    }

    /* 首次调用 → 初始化为居中的滤波值 */
    if (sensor->pos_filtered < 0)
    {
        sensor->pos_filtered = centerPos;
    }

    for (i = 0U; i < 8U; i++)
    {
        /*
         * digital_bits[i] = 0 → 黑线（在线）→ 权重 = 1
         * digital_bits[i] = 1 → 白线（离线）→ 权重 = 0
         *
         * 权重统一用 1，数字型传感器无灰度级，简单计数即可。
         */
        if (sensor->digital_bits[i] == 0U)
        {
            /* 物理位置 = i × 间距 × 100 */
            weighted += (int32_t)i * step;
            total    += 1;
        }
    }

    /* 全白 / 丢线 → 保持上一次有效位置，防止车乱转 */
    if (total == 0)
    {
        return sensor->pos_filtered;
    }

    rawPos = weighted / total;

    /* 限幅 */
    if (rawPos < 0)       { rawPos = 0; }
    if (rawPos > maxPos)  { rawPos = maxPos; }

#if YABO_IR_POSITION_SMOOTHING > 0U
    /* EMA 低通：filtered = filtered*(1-1/N) + rawPos*(1/N) */
    sensor->pos_filtered = sensor->pos_filtered
                 - sensor->pos_filtered / (int32_t)(YABO_IR_POSITION_SMOOTHING)
                 + rawPos                / (int32_t)(YABO_IR_POSITION_SMOOTHING);
#else
    sensor->pos_filtered = rawPos;
#endif

    return sensor->pos_filtered;
}
