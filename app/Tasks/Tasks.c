#include "Tasks.h"
#include "Control/Control.h"             /* direction_pid, speed_loop, yaw_pid, g_graySensor */
#include "PID/PID.h"                     /* PID_Reset, Set_PID, PID_EncoderSpeed_Set */
#include "API_Motor.h"                   /* API_Motor_SetSpeed */
#include "KEY.h"                         /* Key, s_task_select */
#include "Control_Task/Control_Task.h"   /* NonBlockDelay_t */
#include "StepMotor.h"                   /* Stepmotor_Stop */
#include "ICM42688.h"                    /* ICM42688_GetSnapshot */
#include "My_Usart/My_Usart.h"           /* CAM_VALID / CAM_X 宏 */

extern volatile uint32_t g_sys_tick_ms;  /* 系统毫秒 tick，用于计时 */

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

/* ── 反向刹车状态（Task_Run 管理，Task_Stop 设置）── */
static int16_t s_brake_duty  = 0;      /* 刹车占空比（正数=反向）         */
static uint8_t s_brake_ticks = 0U;     /* 刹车剩余 tick 数（20ms/次）     */

uint8_t Task_IsRunning(void)  { return s_task_running; }
uint8_t Task_GetSelect(void)  { return s_task_select; }
uint8_t Task_GetActive(void)  { return s_task_active; }
uint8_t Task_GetPos(void)
{
    if (s_task_active == 3U || s_task_active == 4U) return s_task3_pos;
    return s_task2_pos;
}

/* ── Task_2 圈时输出（Task_2 内部每周期更新，外部只读）── */
static uint32_t s_task2_lap_time_s = 0U;

uint32_t Task_2_GetLapTime(void) { return s_task2_lap_time_s; }

/* ══════════════════════════════════════════════════════════════════════
 * 任务实现
 * ══════════════════════════════════════════════════════════════════════ */

static void Task_1(void)
{

}

/* ══════════════════════════════════════════════════════════════════════
 * Task_2 — 速度环 + 灰度循迹环，顺时针循圆圈黑线跑一圈并计时
 *
 * 终点检测：g_graySensor.digital_bits[2] 和 [5] 同时见黑（== 0）。
 *   圆圈地图上起跑线横穿赛道，跑完一圈回到起点时触发。
 *
 * 计时器：KEY1 启动瞬间开始计时（秒），检测到终点停止。
 *   圈时通过 Task_2_GetLapTime() 读取，供 OLED 显示。
 *
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 * ══════════════════════════════════════════════════════════════════════ */
/* ── Task_2 可调参数 ── */
#define TASK2_START_COOLDOWN  150U  /* 起步冷却，防起跑线误判终点  */
#define TASK2_CONFIRM_CNT     0U    /* 终点消抖次数（0=立即触发）  */

static void Task_2(void)
{
    /* ── 状态机 ── */
    enum { STATE_FOLLOW = 0U, STATE_DONE };

    static uint8_t  s_state      = STATE_FOLLOW;
    static uint8_t  s_last_gen   = 0U;
    static uint8_t  s_cooldown   = 0U;
    static uint8_t  s_confirm    = 0U;
    static uint32_t s_start_tick = 0U;

    if (s_last_gen != s_gen)
    {
        s_last_gen   = s_gen;
        s_state      = STATE_FOLLOW;
        s_cooldown   = (uint8_t)TASK2_START_COOLDOWN;
        s_confirm    = 0U;
        s_start_tick = g_sys_tick_ms;

        PID_EncoderSpeed_Set(&speed_loop, 50.0f, 100.0f, 0.0f, 19.0f);
        Set_PID(&direction_pid,  0.53f, 0.61f, 0.062f);
    }

    switch (s_state)
    {
    case STATE_FOLLOW:
        LineFollow_Output();
        s_task2_lap_time_s = (g_sys_tick_ms - s_start_tick) / 1000U;

        if (s_cooldown > 0U) { s_cooldown--; }
        else if (g_graySensor.digital_bits[4] == 0U &&
                 g_graySensor.digital_bits[5] == 0U &&
                 g_graySensor.digital_bits[6] == 0U &&
                 g_graySensor.digital_bits[7] == 0U)
        {
            if (++s_confirm >= TASK2_CONFIRM_CNT)
            {
                s_state = STATE_DONE;
                Task_Stop(750);
            }
        }
        else { s_confirm = 0U; }
        break;

    case STATE_DONE:
        break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * Task_3 — 小球位置控制：0 → +5 → -5（纯小球，不涉及小车）
 *
 * 摄像头 CAM_X → 小球位置 PID → 步进电机倾斜轨道，
 * 使小球沿半圆弧轨道依次移动到 +5 和 -5 并稳定。
 *
 * 全程小车静止，不碰 DC 电机/速度环/方向环/偏航环。
 *
 * 阶段：
 *   PHASE_TO_P5       — 0 → +5，PID 跟踪直到到达阈值内
 *   PHASE_CONFIRM_P5   — 在 +5 处稳定确认（消抖），然后切目标到 -5
 *   PHASE_TO_N5        — +5 → -5，PID 跟踪
 *   PHASE_CONFIRM_N5   — 在 -5 处稳定确认
 *   PHASE_DONE         — 完成，继续保持当前位置
 *
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 * ══════════════════════════════════════════════════════════════════════ */
/* ── Task_3 可调参数 ── */
#define TASK3_STABLE_THRESHOLD  1.0f  /* 到达判定：|CAM_X - target| ≤ 此值  */
#define TASK3_CONFIRM_TICKS     25U   /* 稳定确认：25×20ms=500ms           */

static void Task_3(void)
{
    enum {
        PHASE_TO_P5 = 0U,
        PHASE_CONFIRM_P5,
        PHASE_TO_N5,
        PHASE_CONFIRM_N5,
        PHASE_DONE
    };

    static uint8_t  s_state       = PHASE_TO_P5;
    static uint8_t  s_last_gen    = 0U;
    static uint8_t  s_confirm_cnt = 0U;

    /* ── KEY1 重新启动 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen    = s_gen;
        s_state       = PHASE_TO_P5;
        s_confirm_cnt = 0U;

        PID_Reset(&ball_pid_pos);
        PID_Reset(&ball_pid_neg);
        Set_PID(&ball_pid_pos, -20.0f, -30.0f, -45.0f);  /* 600RPM快响应：降P减I，重D阻尼 */
        Set_PID(&ball_pid_neg, -10.0f, -40.0f, -55.0f);  /* 600RPM快响应：降P减I，重D阻尼 */
        BallPid_SetTarget(5.0f);         /* 第一阶段：X = +5.0 */
    }

    switch (s_state)
    {
        case PHASE_TO_P5:
            if (CAM_VALID > 0.5f)
            {
                Ball_Move_Control();

                float error = CAM_X - 5.0f;
                if (error < 0.0f) error = -error;

                if (error <= TASK3_STABLE_THRESHOLD)
                {
                    if (++s_confirm_cnt >= TASK3_CONFIRM_TICKS)
                    {
                        s_confirm_cnt = 0U;
                        s_state       = PHASE_CONFIRM_P5;
                        LED_Control(LED1, LED_HIGH);   /* 到达+5，亮灯 */
                    }
                }
                else { s_confirm_cnt = 0U; }
            }
            break;

        case PHASE_CONFIRM_P5:
            if (CAM_VALID > 0.5f)
            {
                Ball_Move_Control();

                if (++s_confirm_cnt >= TASK3_CONFIRM_TICKS)
                {
                    s_confirm_cnt = 0U;
                    PID_Reset(&ball_pid_neg);      /* 切负目标：清 neg I 项 */
                    BallPid_SetTarget(-5.0f);   /* 切换目标：X = -5.0 */
                    s_state = PHASE_TO_N5;
                }
            }
            break;

        case PHASE_TO_N5:
            if (CAM_VALID > 0.5f)
            {
                Ball_Move_Control();

                float error = CAM_X - (-5.0f);
                if (error < 0.0f) error = -error;

                if (error <= TASK3_STABLE_THRESHOLD)
                {
                    if (++s_confirm_cnt >= TASK3_CONFIRM_TICKS)
                    {
                        s_confirm_cnt = 0U;
                        s_state       = PHASE_CONFIRM_N5;
                        LED_Control(LED1, LED_LOW);    /* 到达-5，灭灯 */
                    }
                }
                else { s_confirm_cnt = 0U; }
            }
            break;

        case PHASE_CONFIRM_N5:
            if (CAM_VALID > 0.5f)
            {
                Ball_Move_Control();

                if (++s_confirm_cnt >= TASK3_CONFIRM_TICKS)
                {
                    s_state = PHASE_DONE;
                }
            }
            break;

        case PHASE_DONE:
            /* 完成：继续保持 -5，PID 稳定小球，不碰小车 */
            if (CAM_VALID > 0.5f)
            {
                Ball_Move_Control();
            }
            break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * Task_4 — 双系统协同：灰度循迹 + 小球位置控制
 *
 * 时间线（从 KEY1 按下开始，20ms/tick）：
 *   [0~]      全速巡航 + 灰度循迹
 *   [计时到]  极缓平滑减速至零，无时间限制，停车无感
 *
 * 小车控制：直接巡航循迹 → 计时触发极缓减速停车。
 *
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 * ══════════════════════════════════════════════════════════════════════ */

/* ── Task_4 可调参数 ── */
#define TASK4_CRUISE_TIME_TICKS  500U  /* 触发减速的全局计时（10s）       */
#define TASK4_CRUISE_SPEED       14    /* 巡航速度（编码器单位）          */
#define TASK4_DECEL_STEPS        700U  /* 减速步数，越大停车越平滑        */

static void Task_4(void)
{
    /* ── 状态机 ── */
    enum {
        STATE_CRUISE = 0U,     /* 全速巡航循迹，等计时到                     */
        STATE_DECEL,           /* 极缓平滑减速，无时间压力                   */
        STATE_DONE             /* 停车完成                                  */
    };

    /* ── 静态变量（s_gen 感知 KEY1 重新启动）── */
    static uint8_t  s_state       = STATE_CRUISE;
    static uint8_t  s_last_gen    = 0U;
    static uint16_t s_timer       = 0U;    /* 全局计时器（20ms/tick）        */
    static uint16_t s_decel_step  = 0U;

    /* ── 首次启动 / KEY1 重新启动 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen   = s_gen;
        s_state      = STATE_CRUISE;
        s_timer      = 0U;
        s_decel_step = 0U;

        /* ── 小车：直接巡航速度 ── */
        PID_EncoderSpeed_Set(&speed_loop, 50.0f, 100.0f, 0.0f, TASK4_CRUISE_SPEED);
        Set_PID(&direction_pid,  0.40f, 0.08f, 0.010f);

        /* ── 小球位置环：目标 X=0（始终控制）── */
        Set_PID(&ball_pid_pos, -20.0f, -30.0f, -45.0f);
        Set_PID(&ball_pid_neg, -20.0f, -40.0f, -55.0f);
        BallPid_SetTarget(0.0f);
    }

    /* ── 小球位置控制 — 停车后释放，电机自保持 ── */
    if (CAM_VALID > 0.5f && s_state != STATE_DONE)
    {
        Ball_Move_Control();
    }

    /* ── 全局计时器：每 20ms +1 ── */
    s_timer++;

    /* ── 小车状态机 ── */
    switch (s_state)
    {
    case STATE_CRUISE:
        LineFollow_Output();

        if (s_timer >= TASK4_CRUISE_TIME_TICKS)
        {
            s_state      = STATE_DECEL;
            s_decel_step = 0U;
        }
        break;

    case STATE_DECEL:
    {
        int32_t target = TASK4_CRUISE_SPEED
                       - (int32_t)((uint32_t)s_decel_step * (uint32_t)TASK4_CRUISE_SPEED
                                   / TASK4_DECEL_STEPS);
        if (target < 0) { target = 0; }

        PID_SetTarget(&speed_loop.left,  target);
        PID_SetTarget(&speed_loop.right, target);
        LineFollow_Output();

        s_decel_step++;
        if (s_decel_step >= TASK4_DECEL_STEPS)
        {
            API_Motor_SetSpeed(0, 0);
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
            s_state = STATE_DONE;
        }
        break;
    }

    case STATE_DONE:
        break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * 通用：灰度循迹一圈 + 二次软启动 + 极缓减速停车 + 小球位置控制
 *      （Task_5 / Task_6 共用）
 *
 * 时间线：
 *   [软启动]  二次函数曲线缓升 0→巡航速度（初段极柔）
 *   [巡航]    全速循迹，冷却期后检测终点（灰度[4][5][6][7]全黑=回到A点）
 *   [计时]    检测到A点后非阻塞计时 10s，继续循迹
 *   [减速]    极缓平滑减速至零，无时间限制，停车无感
 *   [停车后]  释放步进电机，自保持
 *
 * 小车控制：二次软启动 → 巡航循迹 → 回A点触发计时 → 极缓减速停车。
 * 小球由外部独立控制。
 *
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 * ══════════════════════════════════════════════════════════════════════ */

/* ── Task_5/6 可调参数（共用）── */
#define TASK_CB_SOFT_START_STEPS  40U   /* 软启动步数，越小越快达巡航        */
#define TASK_CB_CRUISE_SPEED      13    /* 巡航速度（编码器单位）            */
#define TASK_CB_START_COOLDOWN    250U  /* 起步冷却，防起跑线误判终点（5s）  */
#define TASK_CB_CONFIRM_CNT       0U    /* 终点消抖次数（0=立即触发）        */
#define TASK_CB_POST_LAP_TICKS    200U  /* 回A点后继续巡航计时（4s）         */
#define TASK_CB_DECEL_STEPS       700U  /* 减速步数，越大停车越平滑（14s）   */

static void Task_CruiseWithBall(float ball_target)
{
    /* ── 状态机 ── */
    enum {
        STATE_SOFT_START = 0U, /* 二次软启动：速度 0 → CRUISE_SPEED         */
        STATE_CRUISE,          /* 全速巡航 + 冷却 + 检测终点（回到A点）     */
        STATE_POST_LAP,        /* 已过A点，非阻塞计时继续循迹               */
        STATE_DECEL,           /* 极缓平滑减速，无时间压力                   */
        STATE_DONE             /* 停车完成，释放步进电机                     */
    };

    /* ── 静态变量（s_gen 感知 KEY1 重新启动）── */
    static uint8_t  s_state       = STATE_SOFT_START;
    static uint8_t  s_last_gen    = 0U;
    static uint16_t s_timer       = 0U;    /* 非阻塞计时器（20ms/tick）      */
    static uint16_t s_ramp_step   = 0U;
    static uint16_t s_decel_step  = 0U;
    static uint8_t  s_cooldown    = 0U;
    static uint8_t  s_confirm     = 0U;

    /* ── 首次启动 / KEY1 重新启动 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen   = s_gen;
        s_state      = STATE_SOFT_START;
        s_timer      = 0U;
        s_ramp_step  = 0U;
        s_decel_step = 0U;
        s_cooldown   = (uint8_t)TASK_CB_START_COOLDOWN;
        s_confirm    = 0U;

        /* ── 小车：初始速度=0，软启动缓升 ── */
        PID_EncoderSpeed_Set(&speed_loop, 50.0f, 100.0f, 0.0f, 0);
        Set_PID(&direction_pid, 0.50f, 0.15f, 0.010f);

        /* ── 小球位置环 ── */
        Set_PID(&ball_pid_pos, -20.0f, -30.0f, -45.0f);  /* 600RPM快响应：降P减I，重D阻尼 */
        Set_PID(&ball_pid_neg, -20.0f, -40.0f, -55.0f);  /* 600RPM快响应：降P减I，重D阻尼 */
        BallPid_SetTarget(ball_target);
    }

    /* ── 小球位置控制 — 停车后释放，电机自保持 ── */
    if (CAM_VALID > 0.5f && s_state != STATE_DONE)
    {
        Ball_Move_Control();
    }

    /* ── 全局计时器：每 20ms +1 ── */
    s_timer++;

    /* ── 小车状态机 ── */
    switch (s_state)
    {
    case STATE_SOFT_START:
    {
        /* 软启动：速度从 0 线性缓升到 CRUISE_SPEED */
        int32_t target = (int32_t)((uint32_t)s_ramp_step * (uint32_t)TASK_CB_CRUISE_SPEED
                                   / TASK_CB_SOFT_START_STEPS);
        if (target > TASK_CB_CRUISE_SPEED) { target = TASK_CB_CRUISE_SPEED; }

        PID_SetTarget(&speed_loop.left,  target);
        PID_SetTarget(&speed_loop.right, target);
        LineFollow_Output();

        if (s_cooldown > 0U) { s_cooldown--; }

        s_ramp_step++;
        if (s_ramp_step >= TASK_CB_SOFT_START_STEPS)
        {
            PID_SetTarget(&speed_loop.left,  TASK_CB_CRUISE_SPEED);
            PID_SetTarget(&speed_loop.right, TASK_CB_CRUISE_SPEED);
            s_state = STATE_CRUISE;
        }
        break;
    }

    case STATE_CRUISE:
        LineFollow_Output();

        if (s_cooldown > 0U) { s_cooldown--; }
        else if (g_graySensor.digital_bits[4] == 0U &&
                 g_graySensor.digital_bits[5] == 0U &&
                 g_graySensor.digital_bits[6] == 0U &&
                 g_graySensor.digital_bits[7] == 0U)
        {
            if (++s_confirm >= TASK_CB_CONFIRM_CNT)
            {
                /* 检测到A点 → 开始非阻塞计时，继续循迹 */
                s_state = STATE_POST_LAP;
                s_timer = 0U;
            }
        }
        else { s_confirm = 0U; }
        break;

    case STATE_POST_LAP:
        /* 已过A点，继续巡航循迹，等计时器到 10s */
        LineFollow_Output();

        if (s_timer >= TASK_CB_POST_LAP_TICKS)
        {
            s_state      = STATE_DECEL;
            s_decel_step = 0U;
        }
        break;

    case STATE_DECEL:
    {
        /*
         * 极缓平滑减速：线性降速，750 步 × 20ms = 15 秒从巡航到零。
         * 速度环 PID 自然跟踪极缓慢下降的目标，停车无感。
         */
        int32_t target = TASK_CB_CRUISE_SPEED
                       - (int32_t)((uint32_t)s_decel_step * (uint32_t)TASK_CB_CRUISE_SPEED
                                   / TASK_CB_DECEL_STEPS);
        if (target < 0) { target = 0; }

        PID_SetTarget(&speed_loop.left,  target);
        PID_SetTarget(&speed_loop.right, target);
        LineFollow_Output();

        s_decel_step++;
        if (s_decel_step >= TASK_CB_DECEL_STEPS)
        {
            API_Motor_SetSpeed(0, 0);
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
            s_state = STATE_DONE;
        }
        break;
    }

    case STATE_DONE:
        break;
    }
}

/* ── Task_6 小球目标 X 坐标 ── */
int16_t g_task6_ball_target = 0;

/* Task_5：循迹一圈 + 小球稳定在 X=0 */
static void Task_5(void)
{
    Task_CruiseWithBall(0.0f);
}

/* Task_6：循迹一圈 + 小球稳定在指定的 X 坐标处 */
static void Task_6(void)
{
    Task_CruiseWithBall((float)g_task6_ball_target);
}

/*
 * Task_Stop — 急停：停车 + 复位全部 PID + 清除标志。
 *
 * @param brakeDuty  反向刹车占空比。
 *                   0 = 直接停（已平滑减速的场景，如 Task_4）
 *                   >0 = 反向刹车 200ms（10×20ms）后归零（防惯性滑动，如 Task_2）
 *
 * 反向刹车由 Task_Run 自动管理倒计时，任务停止后仍会继续执行直到归零。
 * 下一次 KEY1 启动时自动取消未完成的刹车。
 */
void Task_Stop(int16_t brakeDuty)
{
    s_task_running = 0U;
    s_task_active  = 0U;
    s_task2_pos    = 0U;
    s_task3_pos    = 0U;

    if (brakeDuty > 0)
    {
        s_brake_duty  = brakeDuty;
        s_brake_ticks = 10U;                     /* 10 × 20ms = 200ms 反向刹车 */
        API_Motor_SetSpeed(-brakeDuty, -brakeDuty);
    }
    else
    {
        API_Motor_SetSpeed(0, 0);
    }

    Stepmotor_Stop(STEPMOTOR1);
    PID_Reset(&direction_pid);
    PID_Reset(&speed_loop.left);
    PID_Reset(&speed_loop.right);
    PID_Reset(&yaw_pid);
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
    /* ── 反向刹车倒计时（任务停止后继续执行，直到归零）── */
    if (s_brake_ticks > 0U)
    {
        s_brake_ticks--;
        API_Motor_SetSpeed(-s_brake_duty, -s_brake_duty);
        if (s_brake_ticks == 0U)
        {
            API_Motor_SetSpeed(0, 0);
        }
    }

    /* ── KEY3 = 急停（运行时停车 + PID 全清零）── */
    if (Key == 3U)
    {
        Key = 0U;
        if (s_task_running != 0U)
        {
            Task_Stop(0);    /* 刹车 */
        }
    }

    /* ── KEY1 = 启动 ── */
    if (Key == 1U)
    {
        Key = 0U;
        if (s_task_running == 0U)
        {
            s_brake_ticks = 0U;              /* 取消未完成的反向刹车 */
            API_Motor_SetSpeed(0, 0);
            s_task_running = 1U;
            s_task_active  = s_task_select;
            s_gen++;                                /* 启动代次 +1，各任务用此感知"被重启" */
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
        case 5U: Task_5(); break;
        case 6U: Task_6(); break;
        default: Task_Stop(0); break;
    }
}
