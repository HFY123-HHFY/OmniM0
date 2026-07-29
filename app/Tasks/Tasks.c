#include "Tasks.h"
#include "Control/Control.h"             /* direction_pid, speed_loop, yaw_pid, g_graySensor */
#include "PID/PID.h"                     /* PID_Reset, Set_PID, PID_EncoderSpeed_Set */
#include "API_Motor.h"                   /* API_Motor_SetSpeed */
#include "KEY.h"                         /* Key, s_task_select */
#include "Control_Task/Control_Task.h"   /* NonBlockDelay_t */
#include "StepMotor.h"                   /* StepMotor_Stop */
#include "ICM42688.h"                    /* ICM42688_GetSnapshot */

extern volatile uint32_t g_sys_tick_ms;  /* 系统毫秒 tick，用于计时 */
extern int16_t g_cam_data[];             /* 摄像头数据，g_cam_data[0]=小球X坐标 */

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
static void Task_2(void)
{
    /* ── 起步冷却 + 终点消抖 ── */
    enum { START_COOLDOWN = 50U,        /* 50×20ms=1s, 起跑后冷却      */
           CONFIRM_CNT    = 5U };       /* 5×20ms=100ms 终点消抖确认   */

    /* ── 状态机 ── */
    enum { STATE_FOLLOW = 0U, STATE_DONE };

    static uint8_t  s_state      = STATE_FOLLOW;
    static uint8_t  s_last_gen   = 0U;
    static uint8_t  s_cooldown   = 0U;
    static uint8_t  s_confirm    = 0U;
    static uint32_t s_start_tick = 0U;   /* 起跑时刻 tick             */

    /* ── 首次启动 / KEY1 重新启动 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen   = s_gen;
        s_state      = STATE_FOLLOW;
        s_cooldown   = (uint8_t)START_COOLDOWN;
        s_confirm    = 0U;
        s_start_tick = g_sys_tick_ms;

        /* 速度环：kp=20.0, ki=170.0, kd=0, 目标=20             */
        PID_EncoderSpeed_Set(&speed_loop, 20.0f, 170.0f, 0.0f, 18);
        /* 灰度方向环：kp=2.0, ki=0.5, kd=0.1                    */
        Set_PID(&direction_pid, 0.5f, 0.0f, 0.1f);
    }

    switch (s_state)
    {
    case STATE_FOLLOW:
        LineFollow_Output();

        /* 每周期更新圈时到外部只读变量（OLED 显示用）*/
        s_task2_lap_time_s = (g_sys_tick_ms - s_start_tick) / 1000U;

        /* 起步冷却递减，防止起跑线被误判为终点 */
        if (s_cooldown > 0U) { s_cooldown--; }
        /* 冷却期过后才检测终点：sensor[2] 和 [5] 同时见黑 */
        else if (g_graySensor.digital_bits[2] == 0U &&
                 g_graySensor.digital_bits[5] == 0U)
        {
            if (++s_confirm >= CONFIRM_CNT)
            {
                /* 一圈完成：冻结局时 + 停车 + 复位 PID */
                s_state = STATE_DONE;
                Task_Stop();
            }
        }
        else { s_confirm = 0U; }
        break;

    case STATE_DONE:
        break;   /* 已完成，Task_Stop 已停车，s_task2_lap_time_s 已冻结 */
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * Task_3 — 小球位置控制：0 → +5 → -5
 *
 * 摄像头识别小球 X 轴位置（g_cam_data[0]），位置 PID 控制轨道倾斜电机，
 * 使小球沿半圆弧轨道移动到指定坐标并稳定。
 *
 * 阶段：
 *   PHASE_TO_P5      — 0 → +5，PID 跟踪直到到达阈值内
 *   PHASE_CONFIRM_P5  — 在 +5 处稳定确认（消抖）
 *   PHASE_TO_N5       — +5 → -5，切换目标到 -5
 *   PHASE_CONFIRM_N5  — 在 -5 处稳定确认
 *   PHASE_DONE        — 完成，停车
 *
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 * ══════════════════════════════════════════════════════════════════════ */
static void Task_3(void)
{
    /* ── 状态机枚举 ── */
    enum {
        PHASE_TO_P5 = 0U,
        PHASE_CONFIRM_P5,
        PHASE_TO_N5,
        PHASE_CONFIRM_N5,
        PHASE_DONE
    };

    /* ── 阈值常量 ── */
    enum {
        STABLE_THRESHOLD = 1,       /* 到达判定：|error| ≤ 1（摄像头分辨率级） */
        CONFIRM_TICKS    = 25U      /* 稳定确认：25×20ms = 500ms            */
    };

    /* ── 静态状态变量（由 s_gen 感知重启）── */
    static uint8_t  s_state       = PHASE_TO_P5;
    static uint8_t  s_last_gen    = 0U;
    static uint8_t  s_confirm_cnt = 0U;

    /* ── 首次启动 / KEY1 重新启动 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen    = s_gen;
        s_state       = PHASE_TO_P5;
        s_confirm_cnt = 0U;

        /* 小球位置环结构已由 PID_Control_Init 初始化，这里只需设参数 */
        /* kp=400: 误差 5 单位 → 2000 duty；ki=20: 慢速消静差；kd=15: 阻尼 */
        Set_PID(&ball_pid, 400.0f, 20.0f, 15.0f);
        BallPid_SetTarget(5);           /* 第一阶段目标：X = +5 */
    }

    switch (s_state)
    {
        case PHASE_TO_P5:
            Ball_Move_Control();

            /* 判断是否到达 +5 */
            {
                int16_t error = (int16_t)((int32_t)g_cam_data[0] - 5);
                if (error < 0) error = (int16_t)(-error);   /* abs(error) */

                if (error <= (int16_t)STABLE_THRESHOLD)
                {
                    if (++s_confirm_cnt >= CONFIRM_TICKS)
                    {
                        s_confirm_cnt = 0U;
                        s_state       = PHASE_CONFIRM_P5;
                    }
                }
                else
                {
                    s_confirm_cnt = 0U;   /* 离开阈值则重置消抖计数 */
                }
            }
            break;

        case PHASE_CONFIRM_P5:
            /* 在 +5 稳定后短暂保持（给一次确认周期），然后切目标 */
            Ball_Move_Control();

            if (++s_confirm_cnt >= CONFIRM_TICKS)
            {
                s_confirm_cnt = 0U;
                BallPid_SetTarget(-5);      /* 切换目标：X = -5 */
                s_state = PHASE_TO_N5;
            }
            break;

        case PHASE_TO_N5:
            Ball_Move_Control();

            /* 判断是否到达 -5 */
            {
                int16_t error = (int16_t)((int32_t)g_cam_data[0] - (-5));
                if (error < 0) error = (int16_t)(-error);

                if (error <= (int16_t)STABLE_THRESHOLD)
                {
                    if (++s_confirm_cnt >= CONFIRM_TICKS)
                    {
                        s_confirm_cnt = 0U;
                        s_state       = PHASE_CONFIRM_N5;
                    }
                }
                else
                {
                    s_confirm_cnt = 0U;
                }
            }
            break;

        case PHASE_CONFIRM_N5:
            /* 在 -5 稳定确认后完成任务 */
            Ball_Move_Control();

            if (++s_confirm_cnt >= CONFIRM_TICKS)
            {
                s_state = PHASE_DONE;
                Task_Stop();                /* 停车 + 复位全部 PID */
            }
            break;

        case PHASE_DONE:
            break;   /* 已完成，Task_Stop 已停车 */
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * Task_4 — 双系统协同：灰度循迹 + 小球位置控制
 *
 * 小车方面：
 *   A 点出发，灰度循迹 + 速度环控制，监控 ICM42688 偏航角。
 *   当偏航角变化超过 TASK4_CORNER_YAW_DEG 时，认为到达地图曲线拐角
 *   处 B 点 → 平滑减速停车 → 复位循迹 PID。
 *
 * 摆杆方面：
 *   从 A 到 B 全过程，小球位置环始终控制步进电机，将小球稳定在
 *   坐标 X=0（误差 ±1 单位）。小车停车后继续维持。
 *
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 * ══════════════════════════════════════════════════════════════════════ */

/* 偏航角变化阈值（°）—— 超过此值认为小车已到达曲线拐角 B 点 */
#define TASK4_CORNER_YAW_DEG      90.0f

/* 巡航目标速度（编码器单位/20ms，Task_2 用 18，这里更慢以兼顾小球稳定） */
#define TASK4_CRUISE_SPEED        10

/* 平滑减速参数 */
#define TASK4_DECEL_STEPS         25U     /* 减速步数（25×20ms = 500ms）  */

static void Task_4(void)
{
    /* ── 状态机 ── */
    enum {
        STATE_CRUISE = 0U,     /* 循迹巡航，监控偏航角                 */
        STATE_DECEL,           /* 检测到拐角，平滑减速                 */
        STATE_DONE             /* 已停车，复位循迹 PID，小球继续控制   */
    };

    /* ── 静态变量（s_gen 感知 KEY1 重新启动）── */
    static uint8_t  s_state       = STATE_CRUISE;
    static uint8_t  s_last_gen    = 0U;
    static uint8_t  s_decel_step  = 0U;
    static float    s_yaw_start   = 0.0f;

    /* ── 首次启动 / KEY1 重新启动 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen   = s_gen;
        s_state      = STATE_CRUISE;
        s_decel_step = 0U;

        /* 记录启动时的偏航角作为基准（检测相对变化） */
        {
            ICM42688_Data_t snap;
            ICM42688_GetSnapshot(&snap);
            s_yaw_start = snap.yaw;
        }

        /* ── 小车：低速巡航循迹 ── */
        PID_EncoderSpeed_Set(&speed_loop, 20.0f, 170.0f, 0.0f, TASK4_CRUISE_SPEED);
        Set_PID(&direction_pid, 0.5f, 0.0f, 0.1f);

        /* ── 小球位置环：目标 X=0（始终控制）── */
        Set_PID(&ball_pid, 400.0f, 20.0f, 15.0f);
        BallPid_SetTarget(0);
    }

    /* ════════════════════════════════════════════════════════════════
     * 小球位置控制 — 全阶段持续，不受小车状态影响
     * ════════════════════════════════════════════════════════════════ */
    Ball_Move_Control();

    /* ════════════════════════════════════════════════════════════════
     * 读取当前偏航角，计算相对变化
     * ════════════════════════════════════════════════════════════════ */
    ICM42688_Data_t snap;
    ICM42688_GetSnapshot(&snap);
    float delta_yaw = snap.yaw - s_yaw_start;
    float abs_delta = (delta_yaw < 0.0f) ? -delta_yaw : delta_yaw;

    /* ════════════════════════════════════════════════════════════════
     * 小车状态机
     * ════════════════════════════════════════════════════════════════ */
    switch (s_state)
    {
    case STATE_CRUISE:
        /* 灰度循迹 + 速度环 */
        LineFollow_Output();

        /* 偏航角变化超过阈值 → 到达 B 点拐角，开始减速 */
        if (abs_delta >= TASK4_CORNER_YAW_DEG)
        {
            s_state      = STATE_DECEL;
            s_decel_step = 0U;
        }
        break;

    case STATE_DECEL:
    {
        /*
         * 平滑减速：逐步降低速度目标值。
         * 从 CRUISE_SPEED 线性降到 0，分 DECEL_STEPS 步完成。
         * 每步降 (CRUISE_SPEED / DECEL_STEPS)，避免突然刹车。
         */
        int32_t target = TASK4_CRUISE_SPEED
                       - (int32_t)((uint32_t)s_decel_step * (uint32_t)TASK4_CRUISE_SPEED
                                   / TASK4_DECEL_STEPS);
        if (target < 0) { target = 0; }

        /* 只更新速度目标，不重设 PID 参数 */
        PID_SetTarget(&speed_loop.left,  target);
        PID_SetTarget(&speed_loop.right, target);

        /* 继续循迹（减速过程中仍然跟随灰线） */
        LineFollow_Output();

        s_decel_step++;

        /* 减速完成 → 停车 */
        if (s_decel_step >= TASK4_DECEL_STEPS)
        {
            /* 刹车 + 复位循迹 PID（小球 PID 不动） */
            API_Motor_SetSpeed(0, 0);
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
            s_state = STATE_DONE;
        }
        break;
    }

    case STATE_DONE:
        /* 小车已停，小球位置环继续在 Ball_Move_Control() 中运行 */
        break;
    }
}

static void Task_5(void)
{

}

static void Task_6(void)
{

}

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
    StepMotor_Stop();                   /* 步进电机立即刹车（保持使能，不丢位置） */
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
        default: Task_Stop(); break;
    }
}
