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
static void Task_2(void)
{
    /* ── 起步冷却 + 终点消抖 ── */
    enum { START_COOLDOWN = 150U,        /* 150×20ms=3s, 起跑后冷却      */
           CONFIRM_CNT    = 0U };       /* 5×20ms=100ms 终点消抖确认   */

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

        /* 速度环 */
        PID_EncoderSpeed_Set(&speed_loop, 50.0f, 100.0f, 0.0f, 20.0f); /* 速度环 */
        /* 灰度方向环 */
        Set_PID(&direction_pid,  0.50f, 0.15f, 0.010f);/* 循迹环 */
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
        else if (g_graySensor.digital_bits[4] == 0U &&
                 g_graySensor.digital_bits[5] == 0U &&
                 g_graySensor.digital_bits[6] == 0U)
        {
            if (++s_confirm >= CONFIRM_CNT)
            {
                /* 一圈完成：冻结局时 + 停车 + 复位 PID */
                s_state = STATE_DONE;
                Task_Stop(50);               /* 反向刹车防惯性滑动 */
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
static void Task_3(void)
{
    enum {
        PHASE_TO_P5 = 0U,
        PHASE_CONFIRM_P5,
        PHASE_TO_N5,
        PHASE_CONFIRM_N5,
        PHASE_DONE
    };

    #define STABLE_THRESHOLD   1.0f    /* 到达判定：|error| ≤ 1.0  */
    #define CONFIRM_TICKS      25U     /* 稳定确认：25×20ms=500ms */

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
        Set_PID(&ball_pid_pos, -23.0f, -23.0f, -35.0f);  /* 正目标侧：短力臂 */
        Set_PID(&ball_pid_neg, -30.0f, -22.0f, -45.0f);  /* 负目标侧：长力臂，大kp+kd */
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

                if (error <= STABLE_THRESHOLD)
                {
                    if (++s_confirm_cnt >= CONFIRM_TICKS)
                    {
                        s_confirm_cnt = 0U;
                        s_state       = PHASE_CONFIRM_P5;
                    }
                }
                else { s_confirm_cnt = 0U; }
            }
            break;

        case PHASE_CONFIRM_P5:
            if (CAM_VALID > 0.5f)
            {
                Ball_Move_Control();

                if (++s_confirm_cnt >= CONFIRM_TICKS)
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

                if (error <= STABLE_THRESHOLD)
                {
                    if (++s_confirm_cnt >= CONFIRM_TICKS)
                    {
                        s_confirm_cnt = 0U;
                        s_state       = PHASE_CONFIRM_N5;
                    }
                }
                else { s_confirm_cnt = 0U; }
            }
            break;

        case PHASE_CONFIRM_N5:
            if (CAM_VALID > 0.5f)
            {
                Ball_Move_Control();

                if (++s_confirm_cnt >= CONFIRM_TICKS)
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
 * 小车方面：
 *   A 点出发，灰度循迹 + 速度环控制，监控 ICM42688 偏航角。
 *   起步冷却期内不检测偏航角（防起跑误触发）。
 *   冷却期过后，当偏航角 ≤ TASK4_CORNER_YAW_DEG 时（直走≈0°，
 *   拐到 B 点≈-90°），认为到达曲线拐角 → 平滑减速 → 直行通过 B → 停车。
 *
 * 摆杆方面：
 *   全过程小球位置环始终控制步进电机，将小球稳定在 X=0。
 *
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 * ══════════════════════════════════════════════════════════════════════ */

/* ── 可调参数 ── */
#define TASK4_START_COOLDOWN      150U    /* 起步冷却（150×20ms=3s，防起跑误触发） */
#define TASK4_CORNER_YAW_DEG      -80.0f  /* 偏航角阈值（°），≤此值=到达B点拐角    */
#define TASK4_CRUISE_SPEED        10      /* 巡航速度（编码器单位）                  */
#define TASK4_DECEL_STEPS         25U     /* 减速步数（25×20ms=500ms 平滑减速）     */
#define TASK4_PASS_TICKS          50U     /* 通过B点延时（50×20ms=1s，确保车尾过B） */

static void Task_4(void)
{
    /* ── 状态机 ── */
    enum {
        STATE_CRUISE = 0U,     /* 循迹巡航 + 起步冷却 + 监控偏航角       */
        STATE_DECEL,           /* 检测到拐角 → 平滑减速                  */
        STATE_PASS,            /* 减速完成 → 直行延时，确保车尾通过 B 点  */
        STATE_DONE             /* 通过 B 点，停车，小球继续控制           */
    };

    /* ── 静态变量（s_gen 感知 KEY1 重新启动）── */
    static uint8_t  s_state       = STATE_CRUISE;
    static uint8_t  s_last_gen    = 0U;
    static uint8_t  s_decel_step  = 0U;
    static uint8_t  s_pass_tick   = 0U;
    static uint8_t  s_cooldown    = 0U;

    /* ── 首次启动 / KEY1 重新启动 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen   = s_gen;
        s_state      = STATE_CRUISE;
        s_decel_step = 0U;
        s_pass_tick  = 0U;
        s_cooldown   = (uint8_t)TASK4_START_COOLDOWN;

        /* ── 小车：低速巡航循迹 ── */
        PID_EncoderSpeed_Set(&speed_loop, 50.0f, 100.0f, 0.0f, TASK4_CRUISE_SPEED); /* 速度环 */
        Set_PID(&direction_pid,  0.50f, 0.15f, 0.010f);/* 循迹环 */

        /* ── 小球位置环：目标 X=0（始终控制）── */
        Set_PID(&ball_pid_pos, -23.0f, -23.0f, -35.0f); Set_PID(&ball_pid_neg, -30.0f, -22.0f, -45.0f);
        BallPid_SetTarget(0.0f);
    }

    /* ════════════════════════════════════════════════════════════════
     * 小球位置控制 — 全阶段持续，不受小车状态影响。
     * 丢球时（CAM_VALID=0）跳过控制，保持当前位置，防止追噪声。
     * ════════════════════════════════════════════════════════════════ */
    if (CAM_VALID > 0.5f)
    {
        Ball_Move_Control();
    }

    /* ════════════════════════════════════════════════════════════════
     * 读取当前偏航角（ICM42688 上电归零，直走≈0°，拐弯≈-90°）
     * ════════════════════════════════════════════════════════════════ */
    ICM42688_Data_t snap;
    ICM42688_GetSnapshot(&snap);

    /* ════════════════════════════════════════════════════════════════
     * 小车状态机
     * ════════════════════════════════════════════════════════════════ */
    switch (s_state)
    {
    case STATE_CRUISE:
        /* 灰度循迹 + 速度环 */
        LineFollow_Output();

        /* 起步冷却递减（防起跑误触发偏航角检测） */
        if (s_cooldown > 0U) { s_cooldown--; }
        /* 冷却期过后：偏航角 ≤ 阈值 → 到达 B 点拐角，开始减速 */
        else if (snap.yaw <= TASK4_CORNER_YAW_DEG)
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
         */
        int32_t target = TASK4_CRUISE_SPEED
                       - (int32_t)((uint32_t)s_decel_step * (uint32_t)TASK4_CRUISE_SPEED
                                   / TASK4_DECEL_STEPS);
        if (target < 0) { target = 0; }

        PID_SetTarget(&speed_loop.left,  target);
        PID_SetTarget(&speed_loop.right, target);
        LineFollow_Output();

        s_decel_step++;

        /* 减速完成 → 进入通过阶段（车头已过 B，等车尾也过去） */
        if (s_decel_step >= TASK4_DECEL_STEPS)
        {
            s_state     = STATE_PASS;
            s_pass_tick = 0U;
        }
        break;
    }

    case STATE_PASS:
        /*
         * 直行通过 B 点：减速已完成（target=0），车靠惯性/低速
         * 继续直行 TASK4_PASS_TICKS 帧，确保车尾完全通过 B 点。
         * 期间不再循迹（已过弯道），仅维持小球控制。
         */
        s_pass_tick++;

        if (s_pass_tick >= TASK4_PASS_TICKS)
        {
            /* 通过完成 → 停车 + 复位循迹 PID（小球 PID 不动） */
            API_Motor_SetSpeed(0, 0);
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
            s_state = STATE_DONE;
        }
        break;

    case STATE_DONE:
        /* 小车已停，小球位置环继续在 Ball_Move_Control() 中运行 */
        break;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * 通用：灰度循迹一圈 + 小球位置控制（Task_5 / Task_6 共用）
 *
 * 小车方面：
 *   A 点出发，灰度循迹 + 速度环控制，顺时针循黑线跑一圈。
 *   起步冷却期内不检测终点（防起跑线误触发）。
 *   冷却期过后，灰度传感器 [4][5][6] 同时见黑 → 回到 A 点。
 *   消抖确认后，继续行驶 TASK_CB_PASS_TICKS 帧确保车尾通过 A，
 *   之后停车 + 复位循迹 PID。
 *
 * 摆杆方面：
 *   全过程小球位置环始终控制步进电机，将小球稳定在 ball_target 处。
 *
 * @param ball_target  小球目标 X 坐标（浮点，-12.0~+12.0）
 *
 * 调用者：
 *   Task_5 → ball_target = 0.0f
 *   Task_6 → ball_target = (float)g_task6_ball_target（评委现场指定）
 *
 * 调用频率：TIMG0 ISR 20ms（由 Task_Run 分发）。
 * ══════════════════════════════════════════════════════════════════════ */

/* ── 可调参数 ── */
#define TASK_CB_START_COOLDOWN   150U    /* 起步冷却（150×20ms=3s）          */
#define TASK_CB_CRUISE_SPEED     10      /* 巡航速度（编码器单位）           */
#define TASK_CB_CONFIRM_CNT      0U      /* 终点消抖确认（0=立即触发）       */
#define TASK_CB_PASS_TICKS       10U     /* 通过 A 点延时（10×20ms=200ms）   */

static void Task_CruiseWithBall(float ball_target)
{
    /* ── 状态机 ── */
    enum {
        STATE_CRUISE = 0U,     /* 循迹巡航 + 起步冷却 + 检测终点         */
        STATE_PASS,            /* 检测到终点 → 直行延时，确保通过 A 点   */
        STATE_DONE             /* 通过 A 点，停车，小球继续控制          */
    };

    /* ── 静态变量（s_gen 感知 KEY1 重新启动）── */
    static uint8_t  s_state       = STATE_CRUISE;
    static uint8_t  s_last_gen    = 0U;
    static uint8_t  s_cooldown    = 0U;
    static uint8_t  s_confirm     = 0U;
    static uint8_t  s_pass_tick   = 0U;

    /* ── 首次启动 / KEY1 重新启动 ── */
    if (s_last_gen != s_gen)
    {
        s_last_gen   = s_gen;
        s_state      = STATE_CRUISE;
        s_cooldown   = (uint8_t)TASK_CB_START_COOLDOWN;
        s_confirm    = 0U;
        s_pass_tick  = 0U;

        /* ── 小车：低速巡航循迹 ── */
        PID_EncoderSpeed_Set(&speed_loop, 50.0f, 100.0f, 0.0f, TASK_CB_CRUISE_SPEED);
        Set_PID(&direction_pid, 0.50f, 0.15f, 0.010f);

        /* ── 小球位置环：目标由参数指定 ── */
        Set_PID(&ball_pid_pos, -23.0f, -23.0f, -35.0f); Set_PID(&ball_pid_neg, -30.0f, -22.0f, -45.0f);
        BallPid_SetTarget(ball_target);
    }

    /* ════════════════════════════════════════════════════════════════
     * 小球位置控制 — 全阶段持续。
     * 丢球时（CAM_VALID=0）跳过控制，保持当前位置，防止追噪声。
     * ════════════════════════════════════════════════════════════════ */
    if (CAM_VALID > 0.5f)
    {
        Ball_Move_Control();
    }

    /* ════════════════════════════════════════════════════════════════
     * 小车状态机
     * ════════════════════════════════════════════════════════════════ */
    switch (s_state)
    {
    case STATE_CRUISE:
        LineFollow_Output();

        if (s_cooldown > 0U)
        {
            s_cooldown--;
        }
        else if (g_graySensor.digital_bits[4] == 0U &&
                 g_graySensor.digital_bits[5] == 0U &&
                 g_graySensor.digital_bits[6] == 0U)
        {
            if (++s_confirm >= TASK_CB_CONFIRM_CNT)
            {
                s_state     = STATE_PASS;
                s_pass_tick = 0U;
            }
        }
        else
        {
            s_confirm = 0U;
        }
        break;

    case STATE_PASS:
        LineFollow_Output();
        s_pass_tick++;

        if (s_pass_tick >= TASK_CB_PASS_TICKS)
        {
            API_Motor_SetSpeed(0, 0);
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
            s_state = STATE_DONE;
        }
        break;

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
            Task_Stop(2000);    /* 紧急刹车：反向 2000，200ms 后归零 */
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
        default: Task_Stop(2000); break;    /* 无效任务号 → 急刹车 */
    }
}
