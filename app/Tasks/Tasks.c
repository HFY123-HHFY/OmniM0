#include "Tasks.h"
#include "Control/Control.h"             /* direction_pid, speed_loop, yaw_pid, g_graySensor */
#include "PID/PID.h"                     /* PID_Reset, Set_PID, PID_EncoderSpeed_Set */
#include "API_Motor.h"                   /* API_Motor_SetSpeed */
#include "KEY.h"                         /* Key, s_task_select */
#include "Control_Task/Control_Task.h"   /* NonBlockDelay_t */

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
        default: Task_Stop(); break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * 任务实现
 * ══════════════════════════════════════════════════════════════════════ */

void Task_1(void)
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
void Task_2(void)
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

void Task_3(void)
{

}

void Task_4(void)
{

}
