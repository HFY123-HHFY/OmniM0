#include "Tasks.h"
#include "Control/Control.h"             /* direction_pid, speed_loop, yaw_pid, g_graySensor */
#include "PID/PID.h"                     /* PID_Reset */
#include "API_Motor.h"                   /* API_Motor_SetSpeed */
#include "KEY.h"                         /* Key, s_task_select */
#include "Control_Task/Control_Task.h"   /* NonBlockDelay_t */

/* ══════════════════════════════════════════════════════════════════════
 * 任务链调度框架
 *
 * 按键协议：
 *   KEY1 — 启动：选择完任务后按下，锁存当前 s_task_select 并启动
 *   KEY2 — 选任务：循环切换 s_task_select 1→4（由 KEY.c 在 key_Get 中维护）
 *   KEY3 — 急停：运行中按下 → 停车 + 全部 PID 清零 + 任务复位
 *   KEY4 — 保留
 *
 * Task_Run 在 TIMG0 ISR 20ms 插槽调用。
 * 启动瞬间锁存任务号到 s_task_active，运行中 KEY2 不影响当前任务。
 * ══════════════════════════════════════════════════════════════════════ */

static uint8_t s_task_running = 0U;   /* 0 = 待机，1 = 运行中           */
static uint8_t s_task_active  = 0U;   /* 启动瞬间锁存的任务号 (1-4)     */
static uint8_t s_task2_pos    = 0U;   /* Task_2 当前位置                */
static uint8_t s_task3_pos    = 0U;   /* Task_3/4 共用位置              */
static uint8_t s_gen          = 0U;   /* 启动代次：每次 KEY1 启动 +1     */

uint8_t Task_IsRunning(void)  { return s_task_running; }
uint8_t Task_GetSelect(void)  { return s_task_select; }
uint8_t Task_GetActive(void)  { return s_task_active; }
uint8_t Task_GetPos(void)
{
    if (s_task_active == 3U || s_task_active == 4U) return s_task3_pos;
    return s_task2_pos;
}

/* ── 灰度事件标志位（5ms ISR 置位，任务消费清零）── */
/* 检测器内部 LOCKOUT 保证每次入/离线只触发一次，不会反复置位 */
volatile uint8_t s_gray_enter_fired = 0U;   /* 入线事件 */
volatile uint8_t s_gray_exit_fired  = 0U;   /* 出线事件 */

/*
 * Task_Stop — 急停：停车 + 复位全部 PID + 清灰度标志。
 * 由 KEY3（运行中急停）或任务内部结束条件调用。
 */
void Task_Stop(void)
{
    s_task_running = 0U;
    s_task_active  = 0U;
    s_task2_pos    = 0U;
    s_task3_pos    = 0U;
    API_Motor_SetSpeed(0, 0);
    PID_Reset(&direction_pid);
    PID_Reset(&speed_loop.left);
    PID_Reset(&speed_loop.right);
    PID_Reset(&yaw_pid);
    s_gray_enter_fired = 0U;
    s_gray_exit_fired  = 0U;
}

/*
 * Task_Run — 任务链入口（TIMG0 ISR 20ms 调用一次）。
 *
 * 按键协议（Key 消费后清零，防全局变量持久化导致重复触发）：
 *   KEY3 → 急停（优先级最高，运行中按下立即停车 + PID 清零）
 *   KEY1 → 启动（仅待机时有效，锁存 s_task_select 后启动）
 */
void Task_Run(void)
{
    /* ── KEY3 = 急停（运行时停车 + PID 全清零）── */
    if (Key == 3U)
    {
        Key = 0U;
        if (s_task_running != 0U)
        {
            Task_Stop();
        }
    }

    /* ── KEY1 = 启动 ── */
    if (Key == 1U)
    {
        Key = 0U;
        if (s_task_running == 0U)
        {
            s_task_running = 1U;
            s_task_active  = s_task_select;
            s_gen++;                                /* 启动代次 +1，各任务用此感知"被重启" */
            s_gray_enter_fired = 0U;
            s_gray_exit_fired  = 0U;
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
            PID_Reset(&yaw_pid);
        }
    }

    if (s_task_running == 0U) return;

    /* ── 任务分发 ── */
    switch (s_task_active)
    {
        case 1U: Task_1(); break;
        case 2U: Task_2(); break;
        case 3U: Task_3(); break;
        case 4U: Task_4(); break;
        default: Task_Stop(); break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * 任务实现
 * ══════════════════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════════════════
 * Task_1 — 灰度循迹环 + 速度环控制小车跑正方形黑线套圈
 *
 * 直角转弯逻辑：
 *   顺时针 (CW)  — 右边 3 路 sensor[5][6][7] 同时见黑 → 延时 → 右转
 *   逆时针 (CCW) — 左边 3 路 sensor[0][1][2] 同时见黑 → 延时 → 左转
 *
 * 转弯流程：
 *   检测路口（3 路同时见黑 digital_bits == 0）
 *     → TURN_DELAY_MS 继续循迹直走（让传感器越过黑线交叉区）
 *     → 关闭 PID（速度环 + 灰度方向环），防止积分累积
 *     → 开环差速原地转弯，持续 TURN_PIVOT_MS
 *     → 复位 PID + 冷却期，回到正常循迹
 * ══════════════════════════════════════════════════════════════════════ */
void Task_1(void)
{
    /* ── 转弯参数（实地调试时修改以下 static const 值即可，无需动状态机）── */

    /* 检测到路口后继续循迹直走的时间（ms），让传感器越过黑线交叉区   */
    static const uint16_t TURN_DELAY_MS   = 0U;
    /* 开环原地转弯持续时间（ms），控制转弯角度（~90°），值越大转越多 */
    static const uint16_t TURN_PIVOT_MS   = 420U;
    /* 顺时针右转 — 左轮正转 + 右轮反转 → 车体顺时针原地 pivot       */
    static const int16_t  CW_SPEED_LEFT   = 1000;
    static const int16_t  CW_SPEED_RIGHT  = 2500;
    /* 逆时针左转 — 左轮反转 + 右轮正转 → 车体逆时针原地 pivot       */
    static const int16_t  CCW_SPEED_LEFT  = 2800;
    static const int16_t  CCW_SPEED_RIGHT = 800;
    /* 转弯完成后冷却周期数（×20ms），防止同一路口被重复触发           */
    static const uint8_t  TURN_COOLDOWN   = 15U;

    /* ── 套圈方向（改 DIR_CW 顺时针/ DIR_CCW 逆时针切换）── */
    enum { DIR_CW = 1U, DIR_CCW = 2U };
    static const uint8_t DIR = DIR_CCW;

    /* ── 转弯状态机 ── */
    enum {
        STATE_FOLLOW = 0U,       /* 正常循迹（速度环 + 灰度方向环 PID 全开） */
        STATE_TURN_DELAY,        /* 路口直走延时（PID 仍开，让传感器过线）   */
        STATE_TURN_PIVOT,        /* 开环原地转弯（PID 关闭，差速 pivot）     */
    };

    static uint8_t         s_state    = STATE_FOLLOW;
    static NonBlockDelay_t s_delay;
    static uint8_t         s_cooldown = 0U;
    static uint8_t         s_last_gen = 0U;
    uint8_t turn_trigger = 0U;

    /* ── 首次启动 / KEY1 重新启动时，设置本任务的 PID 参数 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen = s_gen;
        /* 速度环：kp=20.0, ki=170.0, kd=0, 目标=20（编码器单位）      */
        PID_EncoderSpeed_Set(&speed_loop, 20.0f, 170.0f, 0.0f, 20.0f);
        /* 灰度方向环：kp=2.0, ki=0.5, kd=0.1                           */
        Set_PID(&direction_pid, 0.5f, 0.0f, 0.01f);
    }

    /* ── 冷却计数递减（每次 20ms）── */
    if (s_cooldown > 0U) { s_cooldown--; }

    /* ══════════════════════════════════════════════════════════════════
     * 路口检测：3 路灰度灯管同时扫描到黑线（digital_bits[i] == 0）。
     *
     * 顺时针套圈 → 右边 3 路（sensor[5][6][7]）同时见黑 = 右转路口
     * 逆时针套圈 → 左边 3 路（sensor[0][1][2]）同时见黑 = 左转路口
     *
     * 仅对应方向 3 路全部见黑才触发，单路或两路见黑不触发，
     * 天然抗噪（直线上通常只有 1-2 路见黑）。
     * ══════════════════════════════════════════════════════════════════ */
    if (DIR == DIR_CW)
    {
        if (g_graySensor.digital_bits[5] == 0U &&
            g_graySensor.digital_bits[6] == 0U &&
            g_graySensor.digital_bits[7] == 0U)
        {
            turn_trigger = 1U;
        }
    }
    else
    {
        if (g_graySensor.digital_bits[0] == 0U &&
            g_graySensor.digital_bits[1] == 0U &&
            g_graySensor.digital_bits[2] == 0U)
        {
            turn_trigger = 1U;
        }
    }

    /* ══════════════════════════════════════════════════════════════════
     * 状态机
     * ══════════════════════════════════════════════════════════════════ */
    switch (s_state)
    {
    /* ── 正常循迹：速度环 + 灰度方向环融合输出 ── */
    case STATE_FOLLOW:
        LineFollow_Output();
        if (turn_trigger != 0U && s_cooldown == 0U)
        {
            s_state = STATE_TURN_DELAY;
            NonBlockDelay_Start(&s_delay, TURN_DELAY_MS);
        }
        break;

    /* ── 路口直走延时：继续循迹，让传感器物理越过黑线交叉区 ── */
    case STATE_TURN_DELAY:
        LineFollow_Output();
        if (NonBlockDelay_IsDone(&s_delay))
        {
            /* 延时结束，关闭 PID 准备转弯 */
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
            s_state = STATE_TURN_PIVOT;
            NonBlockDelay_Start(&s_delay, TURN_PIVOT_MS);
        }
        break;

    /* ── 开环原地转弯：差速 pivot，PID 关闭 ── */
    case STATE_TURN_PIVOT:
        if (DIR == DIR_CW)
        {
            /* 顺时针右转：左轮前进 + 右轮后退 → 车体顺时针原地 pivot */
            API_Motor_SetSpeed(CW_SPEED_LEFT, CW_SPEED_RIGHT);
        }
        else
        {
            /* 逆时针左转：左轮后退 + 右轮前进 → 车体逆时针原地 pivot */
            API_Motor_SetSpeed(CCW_SPEED_LEFT, CCW_SPEED_RIGHT);
        }
        if (NonBlockDelay_IsDone(&s_delay))
        {
            /* 转弯结束：复位 PID + 启动冷却，防同一路口重复触发 */
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
            s_state    = STATE_FOLLOW;
            s_cooldown = TURN_COOLDOWN;
        }
        break;

    default:
        s_state = STATE_FOLLOW;
        break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * Task_2 — 速度环 + ICM42688 偏航角环修正，控制小车走直线
 *
 * 原理：
 *   速度环（编码器反馈）维持左右轮目标速度一致，
 *   偏航角环（ICM42688 yaw 反馈）补偿轮径差异/地面不平引起的偏航，
 *   两环融合后 → API_Motor_SetSpeed 输出。
 *
 * 偏航角目标 = 0°（直走），小车自动纠偏保持直线。
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 *
 * 核心就是调用 Drive_YawSpeed()，初始化在首次/重启时完成一次即可。
 * ══════════════════════════════════════════════════════════════════════ */
void Task_2(void)
{
    static uint8_t s_last_gen = 0U;

    /* ── 首次启动 / KEY1 重新启动时，设置 PID 参数 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen = s_gen;
        /* 速度环：kp=20.0, ki=170.0, kd=0, 目标=20（编码器单位）  */
        PID_EncoderSpeed_Set(&speed_loop, 20.0f, 170.0f, 0.0f, 25.0f);
        /* 偏航角环：kp=2.0, ki=0.3, kd=0, 目标=0°（走直线）      */
        YawPid_Set(2.2f, 0.3f, 0.0f, 0.0f);
    }

    /* 速度环 + 偏航角环融合输出 */
    Drive_YawSpeed();
}

void Task_3(void)
{

}

void Task_4(void)
{

}
