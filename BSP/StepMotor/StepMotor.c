/*
 * StepMotor.c — 张大头 ZDT_X系列 步进闭环电机驱动
 *
 * 硬件背景：
 *   - 通信方式：UART (API_USART3, 115200 bps)
 *   - 协议格式：地址 + 命令 + 数据... + 0x6B（固定校验字节）
 *   - 0xFD 梯形位置模式数据：方向(1) + 转速(2) + 加速度(1) + 脉冲(4) + 模式(1) + 同步(1)
 *   - 角度 → 脉冲：pulses = angle × 3200 / 360（1.8°电机, 16细分）
 *
 * 架构设计：
 *   - TX：API_USART_WriteByte（G3507_usart.c 已修复，不等 BUSY）
 *   - RX：ISR + 环形缓冲协作模式
 *     ┌─ USART3 ISR ──────────────────────────┐
 *     │  usart_irq_dispatch_by_id 读 FIFO       │
 *     │  → StepMotor_RxPush(byte) → 环形缓冲    │  ← 极快，只入队
 *     └────────────────────────────────────────┘
 *     ┌─ 阻塞命令（Enable/GoHome/GetAngle）────┐
 *     │  StepMotor_RxBuf → 从环形缓冲取字节     │  ← 轮询等待，带超时
 *     └────────────────────────────────────────┘
 *   - 非阻塞命令（SetAngle/MoveBy/Stop）：只发不收
 *
 * 参考：张大头官方 STM32 驱动（已验证通过）
 */

#include "StepMotor.h"
#include "usart.h"              /* API_USART_WriteByte / USART3 */
#include "Delay.h"              /* Delay_ms                        */
#include "Control_Task/Control_Task.h"  /* SysTick_GetMs */

/*===========================================================================
 * 常量与配置
 *===========================================================================*/

#define STEPMOTOR_USART        API_USART3    /* 步进电机串口         */
#define MOTOR_CHECKSUM         0x6BU         /* 协议固定校验字节      */

/* ── 电机参数 ── */
#define MOTOR_PULSES_PER_REV   3200U         /* 1.8°步进角, 16细分   */
#define MOTOR_DEG_PER_PULSE    0.1125f       /* 360/3200              */

/* ── 命令码 ── */
#define CMD_ENABLE        0xF3U
#define CMD_POS_TRAPEZOID 0xFDU   /* 梯形曲线位置模式       */
#define CMD_STOP          0xFEU
#define CMD_READ_POS      0x36U   /* 读取实时位置          */
#define CMD_READ_SPEED    0x35U   /* 读取实时转速          */
#define CMD_READ_STATUS   0x3AU   /* 读取状态标志位        */
#define CMD_SET_ZERO      0x93U   /* 设置单圈回零零点      */
#define CMD_GO_HOME       0x9AU   /* 触发回零              */
#define CMD_CLEAR_ANGLE   0x0AU
#define CMD_RELEASE_STALL 0x0EU

/* ── 应答状态码 ── */
#define RESP_OK           0x02U
#define RESP_ERROR_CMD    0x00U
#define RESP_ERROR_CODE   0xEEU

/* ── 位置模式 ── */
#define POS_RELATIVE      0x00U
#define POS_ABSOLUTE      0x01U

/* ── 方向 ── */
#define DIR_CW            0x00U
#define DIR_CCW           0x01U

/* ── 超时 ── */
#define TIMEOUT_CTRL_MS   500U    /* 控制命令超时          */
#define TIMEOUT_READ_MS   200U    /* 读取命令超时          */

/* ── 默认电机地址 ── */
static uint8_t s_addr = 0x01U;

/* ── 非阻塞运动默认参数 ── */
static float    s_cfg_speed = 600.0f;
static uint8_t  s_cfg_accel = 0x00U;   /* 加速度档位（0=直启） */

/*===========================================================================
 * 环形接收缓冲区（ISR 写入，阻塞命令轮询读取）
 *===========================================================================*/

#define STEPMOTOR_RX_BUF_SIZE  64U

static uint8_t  s_rx_buf[STEPMOTOR_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;   /* ISR 写入位置 */
static uint16_t s_rx_tail = 0U;            /* 主循环读取位置 */

/*===========================================================================
 * 内部辅助
 *===========================================================================*/

/*
 * 发送字节数组，末尾自动追加 0x6B 校验字节。
 */
static void StepMotor_TxBuf(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    for (i = 0U; i < len; i++)
    {
        API_USART_WriteByte(STEPMOTOR_USART, data[i]);
    }
    API_USART_WriteByte(STEPMOTOR_USART, MOTOR_CHECKSUM);
}

/*
 * 从环形缓冲取 len 字节，超时返回 -1。
 *
 * ISR（StepMotor_RxPush）负责把 USART3 收到的字节推入环形缓冲，
 * 此函数从环形缓冲取出。不直接读 USART3 寄存器，避免和 ISR 抢 FIFO。
 */
static int8_t StepMotor_RxBuf(uint8_t *buf, uint8_t len, uint32_t timeoutMs)
{
    uint32_t start = SysTick_GetMs();
    uint8_t  i = 0U;

    while (i < len)
    {
        if (s_rx_tail != s_rx_head)
        {
            /* 环形缓冲有数据 */
            buf[i++] = s_rx_buf[s_rx_tail];
            s_rx_tail = (uint16_t)((s_rx_tail + 1U) % STEPMOTOR_RX_BUF_SIZE);
        }
        else if ((SysTick_GetMs() - start) >= timeoutMs)
        {
            return -1;  /* 超时 */
        }
    }
    return 0;
}

/*
 * 发送命令 + 接收应答，一步完成。
 *
 * @param fc        功能码（0xF3 / 0xFD / 0x36 ...）
 * @param data      数据字节数组（可为 NULL）
 * @param dataLen   数据字节数
 * @param rxBuf     应答缓冲区
 * @param rxLen     期望收到的应答字节数
 * @param timeoutMs 超时
 * @retval  0  成功
 * @retval -1  超时 / 应答不匹配
 */
static int8_t StepMotor_Cmd(uint8_t fc, const uint8_t *data, uint8_t dataLen,
                             uint8_t *rxBuf, uint8_t rxLen, uint32_t timeoutMs)
{
    uint8_t tx[32];
    uint8_t i;

    /* 组装发送帧：地址 + 功能码 + 数据（校验字节由 TxBuf 追加） */
    tx[0] = s_addr;
    tx[1] = fc;
    for (i = 0U; i < dataLen; i++)
    {
        tx[2U + i] = data[i];
    }

    StepMotor_TxBuf(tx, (uint8_t)(2U + dataLen));

    /* 不需要应答则直接返回 */
    if (rxBuf == 0 || rxLen == 0U)
    {
        return 0;
    }

    /* 轮询接收 */
    if (StepMotor_RxBuf(rxBuf, rxLen, timeoutMs) != 0)
    {
        return -1;  /* 超时 */
    }

    /* 校验应答：地址 + 功能码必须匹配 */
    if (rxBuf[0] != s_addr || rxBuf[1] != fc)
    {
        return -1;  /* 应答不匹配 */
    }

    /* 检查状态码（数据首字节） */
    if (rxLen >= 3U && rxBuf[2] != RESP_OK)
    {
        return -1;  /* 命令被拒绝 */
    }

    return 0;
}

/*===========================================================================
 * 初始化
 *===========================================================================*/

void StepMotor_Init(uint8_t addr)
{
    uint16_t i;

    s_addr = addr;
    s_cfg_speed = 600.0f;
    s_cfg_accel = 0x00U;

    /* 清零环形缓冲 */
    s_rx_head = 0U;
    s_rx_tail = 0U;
    for (i = 0U; i < STEPMOTOR_RX_BUF_SIZE; i++)
    {
        s_rx_buf[i] = 0U;
    }
}

/*===========================================================================
 * StepMotor_RxPush — ISR 调用：USART3 收到字节 → 推入环形缓冲
 *
 * 由 Control_Task_USART_Callback 在 USART3 中断中调用。
 * 极快（< 1µs），只做环形缓冲入队，不在 ISR 中解析协议。
 *===========================================================================*/

void StepMotor_RxPush(uint8_t data)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % STEPMOTOR_RX_BUF_SIZE);

    s_rx_buf[s_rx_head] = data;
    s_rx_head = next;

    /* 缓冲满 → 丢弃最老 1 字节（滑动窗口，防止死锁） */
    if (next == s_rx_tail)
    {
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % STEPMOTOR_RX_BUF_SIZE);
    }
}

/*===========================================================================
 * 运动参数配置
 *===========================================================================*/

void StepMotor_ConfigMove(float speed, float accel)
{
    s_cfg_speed = speed;
    /*
     * 加速度映射：float RPM/s → uint8 档位。
     * 参考驱动中 accel 为单字节，0=直启，值越大加速越缓。
     * 这里简化：accel > 0 时取 min(accel/10, 255) 作为档位。
     */
    if (accel <= 0.0f)
    {
        s_cfg_accel = 0x00U;
    }
    else
    {
        uint16_t v = (uint16_t)(accel * 0.1f);
        s_cfg_accel = (v > 254U) ? 0xFEU : (uint8_t)v;
    }
}

/*===========================================================================
 * 使能 / 失能
 *
 * 命令格式：地址 + 0xF3 + 0xAB + 使能状态 + 同步标志 + 0x6B
 *===========================================================================*/

int8_t StepMotor_Enable(void)
{
    uint8_t rx[4];
    const uint8_t d[] = {0xAB, 0x01, 0x00};  /* magic + enable + sync */
    return StepMotor_Cmd(CMD_ENABLE, d, 3U, rx, 4U, TIMEOUT_CTRL_MS);
}

int8_t StepMotor_Disable(void)
{
    const uint8_t d[] = {0xAB, 0x00, 0x00};  /* magic + disable + sync */
    (void)StepMotor_Cmd(CMD_ENABLE, d, 3U, 0, 0U, 0U);
    return 0;
}

/*===========================================================================
 * 梯形曲线位置命令 — 内部共用
 *
 * 参考张大头官方 STM32 驱动，0xFD 命令数据格式：
 *   方向(1) + 转速(2) + 加速度(1) + 脉冲(4) + 模式(1) + 同步(1) = 10 字节
 *
 * 转速 = rpm × 10（大端）
 * 脉冲 = angle × 3200 / 360（大端）
 *===========================================================================*/

static void StepMotor_SendPositionCmd(float angle_deg, uint8_t relAbs)
{
    uint8_t  d[10];
    int32_t  pulses;
    uint16_t spd;
    uint8_t  i;

    /* 角度 → 脉冲 */
    pulses = (int32_t)(angle_deg * (float)MOTOR_PULSES_PER_REV / 360.0f);

    /* 转速 = rpm × 10 */
    spd = (uint16_t)(s_cfg_speed * 10.0f + 0.5f);
    if (spd == 0U) { spd = 10U; }  /* 最小转速 1.0 RPM */

    /* ── 组装 10 字节数据 ── */
    d[0] = (pulses < 0) ? DIR_CCW : DIR_CW;
    if (pulses < 0) { pulses = -pulses; }

    d[1] = (uint8_t)(spd >> 8);                     /* 转速高字节 */
    d[2] = (uint8_t)(spd & 0xFFU);                  /* 转速低字节 */
    d[3] = s_cfg_accel;                             /* 加速度档位  */

    d[4] = (uint8_t)(((uint32_t)pulses >> 24) & 0xFFU);  /* 脉冲 [31:24] */
    d[5] = (uint8_t)(((uint32_t)pulses >> 16) & 0xFFU);  /* 脉冲 [23:16] */
    d[6] = (uint8_t)(((uint32_t)pulses >> 8)  & 0xFFU);  /* 脉冲 [15:8]  */
    d[7] = (uint8_t)((uint32_t)pulses         & 0xFFU);  /* 脉冲 [7:0]   */

    d[8] = relAbs;                                  /* 相对/绝对     */
    d[9] = 0x00U;                                   /* 立即执行      */

    /* ── 组装帧并发送（不等待应答，非阻塞）── */
    {
        uint8_t tx[14];  /* addr + fc + 10 + checksum */
        tx[0] = s_addr;
        tx[1] = CMD_POS_TRAPEZOID;
        for (i = 0U; i < 10U; i++) { tx[2U + i] = d[i]; }
        StepMotor_TxBuf(tx, 12U);
    }
}

/*===========================================================================
 * ② PID 输出接口（非阻塞）
 *===========================================================================*/

void StepMotor_SetAngle(float angle_deg)
{
    StepMotor_SendPositionCmd(angle_deg, POS_ABSOLUTE);
}

void StepMotor_MoveBy(float angle_deg)
{
    StepMotor_SendPositionCmd(angle_deg, POS_RELATIVE);
}

/*===========================================================================
 * 立即停止（非阻塞）
 *
 * 命令格式：地址 + 0xFE + 0x98 + 同步标志 + 0x6B
 *===========================================================================*/

void StepMotor_Stop(void)
{
    const uint8_t d[] = {0x98, 0x00};  /* magic + sync */
    uint8_t tx[4];
    tx[0] = s_addr;
    tx[1] = CMD_STOP;
    tx[2] = d[0];
    tx[3] = d[1];
    StepMotor_TxBuf(tx, 4U);
}

/*===========================================================================
 * 位置与状态读取
 *===========================================================================*/

float StepMotor_GetAngle(void)
{
    uint8_t rx[8];  /* addr + 0x36 + sign(1) + angle(4) + checksum(1) */

    if (StepMotor_Cmd(CMD_READ_POS, 0, 0U, rx, 7U, TIMEOUT_READ_MS) != 0)
    {
        return 0.0f;
    }

    /* 解析：rx[2]=sign, rx[3..6]=angle_raw (big-endian) */
    {
        int32_t raw = ((int32_t)rx[3] << 24) | ((int32_t)rx[4] << 16)
                    | ((int32_t)rx[5] << 8)  |  (int32_t)rx[6];
        if (rx[2] != 0U) { raw = -raw; }
        return (float)raw * 0.1f;  /* ×10 → 度 */
    }
}

float StepMotor_GetSpeed(void)
{
    uint8_t rx[6];  /* addr + 0x35 + sign(1) + speed(2) + checksum(1) */

    if (StepMotor_Cmd(CMD_READ_SPEED, 0, 0U, rx, 5U, TIMEOUT_READ_MS) != 0)
    {
        return 0.0f;
    }

    {
        int16_t raw = (int16_t)(((uint16_t)rx[3] << 8) | (uint16_t)rx[4]);
        if (rx[2] != 0U) { raw = (int16_t)(-raw); }
        return (float)raw * 0.1f;
    }
}

uint8_t StepMotor_GetStatus(void)
{
    uint8_t rx[4];  /* addr + 0x3A + status(1) + checksum(1) */

    if (StepMotor_Cmd(CMD_READ_STATUS, 0, 0U, rx, 4U, TIMEOUT_READ_MS) != 0)
    {
        return 0U;
    }

    return rx[2];
}

uint8_t StepMotor_IsInPosition(void)
{
    return (StepMotor_GetStatus() & STEPMOTOR_STAT_INPOS) ? 1U : 0U;
}

uint8_t StepMotor_IsStalled(void)
{
    uint8_t st = StepMotor_GetStatus();
    return ((st & STEPMOTOR_STAT_STALL) || (st & STEPMOTOR_STAT_PROTECT)) ? 1U : 0U;
}

/*===========================================================================
 * 辅助控制
 *===========================================================================*/

int8_t StepMotor_ClearAngle(void)
{
    const uint8_t d[] = {0x6D};
    uint8_t rx[4];
    return StepMotor_Cmd(CMD_CLEAR_ANGLE, d, 1U, rx, 4U, TIMEOUT_CTRL_MS);
}

int8_t StepMotor_ReleaseStall(void)
{
    const uint8_t d[] = {0x52};
    uint8_t rx[4];
    return StepMotor_Cmd(CMD_RELEASE_STALL, d, 1U, rx, 4U, TIMEOUT_CTRL_MS);
}

/*===========================================================================
 * ⑤ 回零操作
 *
 * 协议参考：张大头官方 STM32 驱动
 *
 * 回零流程：
 *   1. 校准阶段（SetZero，仅一次）：
 *      手动把摆杆转到你想要的机械零点位置 →
 *      调用 StepMotor_SetZero(1) → 零点存入驱动器 Flash。
 *
 *   2. 每次上电（GoHome）：
 *      Enable → GoHome → 电机自动转到零点 → 进入 PID 控制。
 *
 * 回零方向（O_Dir）和回零模式（O_Mode）在驱动器菜单或通过
 * 0x4C 修改回零参数命令设置，不在本驱动中修改。
 *===========================================================================*/

/*
 * 设置单圈回零零点 — 将当前物理位置保存为绝对零点。
 *
 * 命令格式：地址 + 0x93 + 0x88 + 存储标志 + 0x6B
 *
 * @param saveToFlash  0x01=存入 Flash（掉电保持，推荐）
 *                     0x00=仅本次上电有效
 * @retval  0  成功
 * @retval -1  超时/应答错误
 */
int8_t StepMotor_SetZero(uint8_t saveToFlash)
{
    const uint8_t d[] = {0x88, (saveToFlash != 0U) ? 0x01U : 0x00U};
    uint8_t rx[4];
    return StepMotor_Cmd(CMD_SET_ZERO, d, 2U, rx, 4U, TIMEOUT_CTRL_MS);
}

/*
 * 触发单圈回零并阻塞等待完成。
 *
 * 命令格式：地址 + 0x9A + 回零模式 + 同步标志 + 0x6B
 *   回零模式：0x00 = 单圈就近回零（方向由驱动器 O_Dir 菜单决定）
 *
 * 发送指令后，每 10ms 读取一次电机状态（0x3A），
 * 检查到位标志（bit1）且未堵转（bit2=0），即认为回零成功。
 *
 * @param timeoutMs  最大等待时间（ms），默认 5000
 * @retval  0  回零成功
 * @retval -1  超时 / 堵转 / 指令被拒绝
 */
int8_t StepMotor_GoHome(uint32_t timeoutMs)
{
    const uint8_t d[] = {0x00, 0x00};  /* 单圈就近模式 + 立即执行 */
    uint8_t rx[4];
    int8_t  ret;
    uint32_t deadline;

    /* ── 发送回零指令 ── */
    ret = StepMotor_Cmd(CMD_GO_HOME, d, 2U, rx, 4U, 1000U);
    if (ret != 0)
    {
        return -1;  /* 超时 / 应答不匹配 / 被拒绝（rx[2]==0xE2 表示条件不满足） */
    }

    /* ── 轮询等待回零完成 ── */
    deadline = timeoutMs / 10U;
    if (deadline == 0U) { deadline = 1U; }

    while (deadline > 0U)
    {
        uint8_t st = StepMotor_GetStatus();

        /* 到位 且 未堵转 → 回零成功 */
        if ((st & STEPMOTOR_STAT_INPOS) && !(st & STEPMOTOR_STAT_STALL))
        {
            return 0;
        }

        /* 堵转保护触发 → 回零失败 */
        if (st & STEPMOTOR_STAT_PROTECT)
        {
            return -1;
        }

        Delay_ms(10U);
        deadline--;
    }

    return -1;  /* 超时 */
}
