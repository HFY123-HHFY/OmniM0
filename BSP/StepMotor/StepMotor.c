/*
 * StepMotor.c — Emm42 V5.0 步进闭环电机驱动
 *
 * 硬件背景：
 *   - 通信方式：UART (API_USART2, 115200 bps)
 *   - 协议格式：地址 + 命令 + 数据... + 0x6B（固定校验字节）
 *   - 0xFD 梯形位置模式数据：方向(1) + 转速(2) + 加速度(1) + 脉冲(4) + 模式(1) + 同步(1)
 *   - 角度 → 脉冲：pulses = angle × 3200 / 360（1.8°电机, 16细分）
 *
 * 架构设计：
 *   - TX：API_USART_WriteByte（G3507_usart.c 已修复，不等 BUSY）
 *   - RX：ISR + 环形缓冲协作模式
 *     ┌─ USART2 ISR ──────────────────────────┐
 *     │  usart_irq_dispatch_by_id 读 FIFO       │
 *     │  → StepMotor_RxPush(byte) → 环形缓冲    │  ← 极快，只入队
 *     └────────────────────────────────────────┘
 *     ┌─ 阻塞命令（Enable/GoHome/ReadAngle）───┐
 *     │  StepMotor_RxBuf → 从环形缓冲取字节     │  ← 轮询等待，带超时
 *     └────────────────────────────────────────┘
 *   - 非阻塞命令（SetAngle/MoveBy/Stop）：只发不收
 */

#include "StepMotor.h"
#include "usart.h"              /* API_USART_WriteByte */
#include "Delay.h"              /* Delay_ms            */
#include "Control_Task/Control_Task.h"  /* SysTick_GetMs */
#include "LED.h"                /* LED_Control — 校准完成亮灯 */

/*===========================================================================
 * 常量与配置
 *===========================================================================*/

#define STEPMOTOR_USART        API_USART2    /* 步进电机串口         */
#define MOTOR_CHECKSUM         0x6BU         /* 协议固定校验字节      */

/* ── 电机参数 ── */
#define MOTOR_PULSES_PER_REV   3200U         /* 1.8°步进角, 16细分   */

/* ── 命令码 ── */
#define CMD_CALIBRATE     0x06U   /* 编码器校准                */
#define CMD_CLEAR_ANGLE   0x0AU   /* 当前位置清零              */
#define CMD_RELEASE_STALL 0x0EU   /* 解除堵转保护              */
#define CMD_READ_SPEED    0x35U   /* 读取实时转速              */
#define CMD_READ_POS      0x36U   /* 读取实时位置              */
#define CMD_READ_STATUS   0x3AU   /* 读取状态标志位            */
#define CMD_SET_ZERO      0x93U   /* 设置单圈回零零点          */
#define CMD_GO_HOME       0x9AU   /* 触发回零                  */
#define CMD_ENABLE        0xF3U   /* 使能/失能                 */
#define CMD_POS_TRAPEZOID 0xFDU   /* 梯形曲线位置模式          */
#define CMD_STOP          0xFEU   /* 急停                      */

/* ── 应答状态码 ── */
#define RESP_OK           0x02U

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
 */
static int8_t StepMotor_RxBuf(uint8_t *buf, uint8_t len, uint32_t timeoutMs)
{
    uint32_t start = SysTick_GetMs();
    uint8_t  i = 0U;

    while (i < len)
    {
        if (s_rx_tail != s_rx_head)
        {
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
 * 发送命令 + 接收应答（内部通用）。
 *
 * @param checkResp  1=校验 rx[2]==0x02（控制命令），0=不校验（读取命令）
 */
static int8_t StepMotor_CmdEx(uint8_t fc, const uint8_t *data, uint8_t dataLen,
                               uint8_t *rxBuf, uint8_t rxLen, uint32_t timeoutMs,
                               uint8_t checkResp)
{
    uint8_t tx[32];
    uint8_t i;

    tx[0] = s_addr;
    tx[1] = fc;
    for (i = 0U; i < dataLen; i++)
    {
        tx[2U + i] = data[i];
    }

    StepMotor_TxBuf(tx, (uint8_t)(2U + dataLen));

    if (rxBuf == 0 || rxLen == 0U) { return 0; }

    if (StepMotor_RxBuf(rxBuf, rxLen, timeoutMs) != 0) { return -1; }

    if (rxBuf[0] != s_addr || rxBuf[1] != fc) { return -1; }

    if ((checkResp != 0U) && (rxLen >= 3U) && (rxBuf[2] != RESP_OK))
    {
        return -1;
    }

    return 0;
}

/* 控制命令：校验 rx[2]==0x02 */
static int8_t StepMotor_Cmd(uint8_t fc, const uint8_t *data, uint8_t dataLen,
                             uint8_t *rxBuf, uint8_t rxLen, uint32_t timeoutMs)
{
    return StepMotor_CmdEx(fc, data, dataLen, rxBuf, rxLen, timeoutMs, 1U);
}

/* 读取命令：不校验 rx[2]（那是数据） */
static int8_t StepMotor_Read(uint8_t fc, uint8_t *rxBuf, uint8_t rxLen, uint32_t timeoutMs)
{
    return StepMotor_CmdEx(fc, 0, 0U, rxBuf, rxLen, timeoutMs, 0U);
}

/* ── 内部：int8_t → MotorErrCode ── */
static MotorErrCode to_err(int8_t ret)
{
    if (ret == 0) return MOTOR_OK;
    return MOTOR_ERR_NONE;
}

/*===========================================================================
 * ① 模块初始化
 *===========================================================================*/

/*
 * 关闭 RX 中断残留，清空环形缓冲。
 * 在 API_USART_Init 之后、其他 Stepmotor_* 之前调用。
 */
void Stepmotor_Init(void)
{
    uint16_t i;

    s_addr      = 0x01U;
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
 * Stepmotor_CalibrateOnce — 设零点（仅需执行一次！）
 *
 * 前提：驱动板已通过自带菜单完成编码器校准，此函数不再调用 Calibrate(0x06)。
 *
 * 操作步骤：
 *   1. 先断电，手动把电机拨到你想要的机械零点位置
 *   2. 上电 → 自动 Enable → SetOrigin 存入 Flash → GoHome 验证 → LED2 常亮
 *   3. 断电 → 注释掉此函数 → 恢复 BootInit → 重新烧录
 *
 * LED 指示：
 *   通信失败        → 灯全灭（检查接线/驱动板供电）
 *   SetOrigin 失败  → LED1 慢闪（2s 周期）
 *   GoHome 验证失败 → LED1+LED2 交替闪
 *   ★ 全部完成     → LED2 常亮
 *===========================================================================*/
void Stepmotor_CalibrateOnce(void)
{
    MotorErrCode e;

    /* ── ① 清理环形缓冲 ── */
    Stepmotor_Init();

    /* ── ② 等驱动板上线（失败 → 卡死，灯全灭）── */
    if (Stepmotor_SelfTest(STEPMOTOR1) != MOTOR_OK) { while (1); }
    if (Stepmotor_Enable(STEPMOTOR1)   != MOTOR_OK) { while (1); }

    /*
     * ── ③ 保存当前机械位置为零点（存入 Flash）──
     *
     * ★ 上电前你已经手动把电机拨到了想要的零点位置。
     *    这里直接保存，不调 Calibrate（驱动板已自带编码器校准）。
     *    失败自动重试 3 次，每次间隔 2s。
     */
    {
        uint8_t retry;
        for (retry = 0U; retry < 3U; retry++)
        {
            e = Stepmotor_SetOrigin(STEPMOTOR1, 1U);
            if (e == MOTOR_OK) break;
            Delay_ms(2000);
        }
    }

    if (e != MOTOR_OK)
    {
        /* SetOrigin 3 次都失败 → LED1 慢闪 */
        while (1)
        {
            LED_Control(LED1, LED_HIGH); Delay_ms(1000);
            LED_Control(LED1, LED_LOW);  Delay_ms(1000);
        }
    }

    /* ── ④ 等 Flash 写入完成 ── */
    Delay_ms(1000);

    /* ── ⑤ 验证回零（失败 → LED1+LED2 交替闪）── */
    e = Stepmotor_GoHome(STEPMOTOR1, 10000U);
    if (e != MOTOR_OK)
    {
        while (1)
        {
            LED_Control(LED1, LED_HIGH); LED_Control(LED2, LED_LOW);
            Delay_ms(500);
            LED_Control(LED1, LED_LOW);  LED_Control(LED2, LED_HIGH);
            Delay_ms(500);
        }
    }

    /*
     * ── ⑥ ★ 全部完成，LED2 常亮 ──
     * 断电 → 注释掉此函数 → 恢复 BootInit → 重新烧录。
     */
    LED_Control(LED1, LED_LOW);
    LED_Control(LED2, LED_HIGH);

    while (1);
}

/*===========================================================================
 * Stepmotor_BootInit — 每次上电的电机初始化（等驱动板上线 + 使能 + 回零）
 *
 * MCU 上电比驱动板快，需要 SelfTest 重试等待驱动板完成自检。
 * 连接成功后使能电机并自动回零到此前 CalibrateOnce 保存的零点。
 *
 * @retval MOTOR_OK       初始化成功，电机已在零点，可以进入 PID 控制
 * @retval MOTOR_ERR_NONE 驱动板无应答（5s 内未上线，检查接线/供电）
 *===========================================================================*/
MotorErrCode Stepmotor_BootInit(void)
{
    MotorErrCode e = MOTOR_ERR_NONE;
    uint32_t start = SysTick_GetMs();

    Stepmotor_Init();

    /* 等驱动板上线，最多 5 秒（每 100ms 重试一次） */
    while ((SysTick_GetMs() - start) < 5000U)
    {
        e = Stepmotor_SelfTest(STEPMOTOR1);
        if (e == MOTOR_OK)
        {
            e = Stepmotor_Enable(STEPMOTOR1);
            if (e == MOTOR_OK) break;
        }
        Delay_ms(100);
    }

    if (e != MOTOR_OK)
    {
        return MOTOR_ERR_NONE;  /* 驱动板未上线 */
    }

    /* 自动回零到此前 CalibrateOnce 保存的零点 */
    return Stepmotor_GoHome(STEPMOTOR1, 8000U);
}

/*===========================================================================
 * StepMotor_RxPush — ISR 调用：USART 收到字节 → 推入环形缓冲
 *===========================================================================*/

void StepMotor_RxPush(uint8_t data)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % STEPMOTOR_RX_BUF_SIZE);

    s_rx_buf[s_rx_head] = data;
    s_rx_head = next;

    if (next == s_rx_tail)
    {
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % STEPMOTOR_RX_BUF_SIZE);
    }
}

/*===========================================================================
 * ② 通信检测与使能
 *===========================================================================*/

MotorErrCode Stepmotor_SelfTest(uint8_t id)
{
    uint8_t rx[4];

    (void)id;  /* 当前仅支持单电机，id 预留 */
    s_addr = id;

    /* 读一次状态，能收到应答即通信正常 */
    if (StepMotor_Read(CMD_READ_STATUS, rx, 4U, TIMEOUT_READ_MS) != 0)
    {
        return MOTOR_ERR_NONE;
    }
    return MOTOR_OK;
}

MotorErrCode Stepmotor_Enable(uint8_t id)
{
    uint8_t rx[4];
    const uint8_t d[] = {0xAB, 0x01, 0x00};

    s_addr = id;
    return to_err(StepMotor_Cmd(CMD_ENABLE, d, 3U, rx, 4U, TIMEOUT_CTRL_MS));
}

MotorErrCode Stepmotor_Disable(uint8_t id)
{
    const uint8_t d[] = {0xAB, 0x00, 0x00};

    s_addr = id;
    (void)StepMotor_Cmd(CMD_ENABLE, d, 3U, 0, 0U, 0U);
    return MOTOR_OK;
}

MotorErrCode Stepmotor_Stop(uint8_t id)
{
    const uint8_t d[] = {0x98, 0x00};
    uint8_t tx[4];

    s_addr = id;
    tx[0] = s_addr;
    tx[1] = CMD_STOP;
    tx[2] = d[0];
    tx[3] = d[1];
    StepMotor_TxBuf(tx, 4U);
    return MOTOR_OK;
}

/*===========================================================================
 * ③ 首次校准（仅一次，电机必须空载！）
 *===========================================================================*/

/*
 * 编码器校准：电机自动旋转完成编码器对齐。
 *
 * 命令格式：地址 + 0x06 + 0x45 + 0x6B
 * 发送后电机会自动旋转数秒，完成后自动停止。
 */
MotorErrCode Stepmotor_Calibrate(uint8_t id, uint32_t timeoutMs)
{
    const uint8_t d[] = {0x45};
    uint8_t rx[4];
    uint32_t deadline;

    s_addr = id;

    /* ── 发送校准指令 ── */
    if (StepMotor_Cmd(CMD_CALIBRATE, d, 1U, rx, 4U, 1000U) != 0)
    {
        return MOTOR_ERR_NONE;
    }

    /* ── 等待校准完成（电机会自动旋转数秒后停止）── */
    deadline = SysTick_GetMs() + timeoutMs;

    while (SysTick_GetMs() < deadline)
    {
        uint8_t st = Stepmotor_ReadStatus(id);

        /* 到位且未堵转 → 校准完成 */
        if ((st & MOTOR_STAT_IN_POS) && !(st & MOTOR_STAT_STALL_NOW))
        {
            return MOTOR_OK;
        }

        /* 堵转锁死 → 校准失败 */
        if (st & MOTOR_STAT_STALL_LOCK)
        {
            return MOTOR_ERR_STALL;
        }

        Delay_ms(100U);  /* 校准期间 100ms 查询一次即可 */
    }

    return MOTOR_ERR_TIMEOUT;
}

/*
 * 将电机当前位置保存为回零原点。
 *
 * 命令格式：地址 + 0x93 + 0x88 + 存储标志 + 0x6B
 */
MotorErrCode Stepmotor_SetOrigin(uint8_t id, uint8_t saveToFlash)
{
    const uint8_t d[] = {0x88, (saveToFlash != 0U) ? 0x01U : 0x00U};
    uint8_t rx[4];

    s_addr = id;
    return to_err(StepMotor_Cmd(CMD_SET_ZERO, d, 2U, rx, 4U, TIMEOUT_CTRL_MS));
}

/*
 * 触发回零并阻塞等待完成。
 *
 * 命令格式：地址 + 0x9A + 回零模式 + 同步标志 + 0x6B
 */
MotorErrCode Stepmotor_GoHome(uint8_t id, uint32_t timeoutMs)
{
    const uint8_t d[] = {0x00, 0x00};  /* 单圈就近模式 + 立即执行 */
    uint8_t rx[4];
    uint32_t deadline;

    s_addr = id;

    /* ── 发送回零指令（1s 内必须收到应答）── */
    if (StepMotor_Cmd(CMD_GO_HOME, d, 2U, rx, 4U, 1000U) != 0)
    {
        return MOTOR_ERR_HOME_FAIL;  /* 应答错误 / 未保存过零点 */
    }

    /* ── 真实时间超时轮询 ── */
    deadline = SysTick_GetMs() + timeoutMs;

    while (SysTick_GetMs() < deadline)
    {
        uint8_t st = Stepmotor_ReadStatus(id);

        if ((st & MOTOR_STAT_IN_POS) && !(st & MOTOR_STAT_STALL_NOW))
        {
            return MOTOR_OK;
        }

        if (st & MOTOR_STAT_STALL_LOCK)
        {
            return MOTOR_ERR_STALL;
        }

        Delay_ms(3U);
    }

    return MOTOR_ERR_TIMEOUT;
}

MotorErrCode Stepmotor_ResetStall(uint8_t id)
{
    const uint8_t d[] = {0x52};
    uint8_t rx[4];

    s_addr = id;
    return to_err(StepMotor_Cmd(CMD_RELEASE_STALL, d, 1U, rx, 4U, TIMEOUT_CTRL_MS));
}

/*===========================================================================
 * ④ 运动控制
 *===========================================================================*/

void Stepmotor_ConfigMove(float speed, float accel)
{
    s_cfg_speed = speed;
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

/*
 * 梯形曲线位置命令 — 内部共用
 */
static void StepMotor_SendPositionCmdEx(float angle_deg, uint16_t rpm,
                                         uint8_t accel, uint8_t relAbs)
{
    uint8_t  d[10];
    int32_t  pulses;
    uint16_t spd;
    uint8_t  i;

    /* 转速：外部指定或沿用 ConfigMove */
    if (rpm == 0U)
    {
        spd = (uint16_t)(s_cfg_speed * 10.0f + 0.5f);
    }
    else
    {
        spd = rpm * 10U;
    }
    if (spd == 0U) { spd = 10U; }

    /* 加速度：外部指定或沿用 ConfigMove */
    if (accel == 0U) { accel = s_cfg_accel; }

    /* 角度 → 脉冲 */
    pulses = (int32_t)(angle_deg * (float)MOTOR_PULSES_PER_REV / 360.0f);

    /* ── 组装 10 字节数据 ── */
    d[0] = (pulses < 0) ? DIR_CCW : DIR_CW;
    if (pulses < 0) { pulses = -pulses; }

    d[1] = (uint8_t)(spd >> 8);
    d[2] = (uint8_t)(spd & 0xFFU);
    d[3] = accel;

    d[4] = (uint8_t)(((uint32_t)pulses >> 24) & 0xFFU);
    d[5] = (uint8_t)(((uint32_t)pulses >> 16) & 0xFFU);
    d[6] = (uint8_t)(((uint32_t)pulses >> 8)  & 0xFFU);
    d[7] = (uint8_t)((uint32_t)pulses         & 0xFFU);

    d[8] = relAbs;
    d[9] = 0x00U;  /* 立即执行 */

    /* ── 发送（非阻塞，不等应答）── */
    {
        uint8_t tx[14];
        tx[0] = s_addr;
        tx[1] = CMD_POS_TRAPEZOID;
        for (i = 0U; i < 10U; i++) { tx[2U + i] = d[i]; }
        StepMotor_TxBuf(tx, 12U);
    }
}

/* ── 内部：角度非法值安全裁切 ── */
static float clamp_angle(float a)
{
    if (a >  720.0f) a =  720.0f;
    if (a < -720.0f) a = -720.0f;
    return a;
}

void Stepmotor_MoveTo(uint8_t id, float angle, uint16_t rpm,
                      uint8_t accel, uint8_t mode)
{
    s_addr = id;
    StepMotor_SendPositionCmdEx(clamp_angle(angle), rpm, accel, mode);
}

MotorErrCode Stepmotor_GoTo(uint8_t id, float angle, uint16_t rpm,
                            uint8_t accel, uint8_t mode, uint32_t timeoutMs)
{
    uint32_t deadline;

    s_addr = id;
    StepMotor_SendPositionCmdEx(clamp_angle(angle), rpm, accel, mode);

    /* ── 轮询等到位 ── */
    deadline = SysTick_GetMs() + timeoutMs;

    while (SysTick_GetMs() < deadline)
    {
        uint8_t st = Stepmotor_ReadStatus(id);

        if ((st & MOTOR_STAT_IN_POS) && !(st & MOTOR_STAT_STALL_NOW))
        {
            return MOTOR_OK;
        }

        if (st & MOTOR_STAT_STALL_LOCK)
        {
            return MOTOR_ERR_STALL;
        }

        Delay_ms(5U);
    }

    return MOTOR_ERR_TIMEOUT;
}

/* ── 便捷函数（兼容旧 PID 接口）── */

void Stepmotor_SetAngle(uint8_t id, float angle_deg)
{
    s_addr = id;
    StepMotor_SendPositionCmdEx(clamp_angle(angle_deg), 0U, 0U, MOTOR_MODE_ABS);
}

void Stepmotor_MoveBy(uint8_t id, float angle_deg)
{
    s_addr = id;
    StepMotor_SendPositionCmdEx(angle_deg, 0U, 0U, MOTOR_MODE_REL);
}

/*===========================================================================
 * ⑤ 状态查询
 *===========================================================================*/

float Stepmotor_ReadAngle(uint8_t id)
{
    uint8_t rx[8];

    s_addr = id;

    if (StepMotor_Read(CMD_READ_POS, rx, 7U, TIMEOUT_READ_MS) != 0)
    {
        return 0.0f;
    }

    {
        int32_t raw = ((int32_t)rx[3] << 24) | ((int32_t)rx[4] << 16)
                    | ((int32_t)rx[5] << 8)  |  (int32_t)rx[6];
        if (rx[2] != 0U) { raw = -raw; }
        return (float)raw * 0.1f;
    }
}

float Stepmotor_ReadSpeed(uint8_t id)
{
    uint8_t rx[6];

    s_addr = id;

    if (StepMotor_Read(CMD_READ_SPEED, rx, 5U, TIMEOUT_READ_MS) != 0)
    {
        return 0.0f;
    }

    {
        int16_t raw = (int16_t)(((uint16_t)rx[3] << 8) | (uint16_t)rx[4]);
        if (rx[2] != 0U) { raw = (int16_t)(-raw); }
        return (float)raw * 0.1f;
    }
}

uint8_t Stepmotor_ReadStatus(uint8_t id)
{
    uint8_t rx[4];

    s_addr = id;

    if (StepMotor_Read(CMD_READ_STATUS, rx, 4U, TIMEOUT_READ_MS) != 0)
    {
        return 0U;
    }

    return rx[2];
}

