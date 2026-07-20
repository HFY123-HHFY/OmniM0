#include "Tasks.h"
#include "Buzzer.h"             /* Buzzer_Alert */
#include "Control/Control.h"   /* YawPid_Calc, speed_loop, Encoder1/2, TB6612 */
#include "Control_Task/Control_Task.h"  /* NonBlockDelay_t */
#include "jy61p.h"     /* JY61P_GetYawFiltered */
#include "PID/PID.h"
#include "TB6612.h"
#include "KEY.h"

/* ══════════════════════════════════════════════════════════════════════
 * 任务链调度 — KEY1 启动/停止，KEY2 循环选择任务 (1-4)
 *
 * s_task_select 由 KEY.c 维护（KEY2 循环 1→4，同 KEY3 设圈数模式）。
 * 启动时把 s_task_select 锁存到 s_task_active，运行中按 KEY2 不会切换任务。
 * Task_Run 在 TIMG0 ISR 20ms 插槽调用（与 Control_Run 同位置）。
 * ══════════════════════════════════════════════════════════════════════ */

static uint8_t s_task_running = 0U;   /* 0 = 待机，1 = 运行中           */
static uint8_t s_task_active  = 0U;   /* 启动瞬间锁存的任务号 (1-4)     */
static uint8_t s_task2_pos    = 0U;   /* Task_2 当前位置: 1=A,2=B,3=C,4=D,5=DONE */
static uint8_t s_task3_pos    = 0U;   /* Task_3 当前位置: 1=A,2=C,3=B,4=D,5=DONE */
static uint8_t s_gen          = 0U;   /* 启动代次：每次 KEY1 启动 +1，任务用它感知重启 */

uint8_t Task_IsRunning(void)  { return s_task_running; }
uint8_t Task_GetSelect(void)  { return s_task_select; }
uint8_t Task_GetActive(void)  { return s_task_active; }
uint8_t Task_GetPos(void) {
    if (s_task_active == 3U || s_task_active == 4U) return s_task3_pos;   /* Task3/4 共用位置 */
    return s_task2_pos;
}

/*
 * Task_Stop — 停止当前任务：停车 + 复位所有 PID。
 * 任务函数内部达成结束条件时也可直接调用。
 */
void Task_Stop(void)
{
    s_task_running = 0U;
    s_task_active  = 0U;
    s_task2_pos    = 0U;
    s_task3_pos    = 0U;
    TB6612_SetSpeed(0, 0);
    PID_Reset(&direction_pid);
    PID_Reset(&speed_loop.left);
    PID_Reset(&speed_loop.right);
    s_gray_enter_fired = 0U;
    s_gray_exit_fired  = 0U;
}

/*
 * Task_Run — 任务链入口（TIMG0 ISR 20ms 调用一次）。
 *
 * KEY1 消费模式（读后清零，防全局变量持久化导致重复触发）：
 *   待机时按下 → 锁存 s_task_select 并启动；
 *   运行时按下 → 急停（Task_Stop）。
 */
void Task_Run(void)
{
    /* ── 按键：KEY1 = 启动 / 停止（toggle）── */
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
        }
        else
        {
            Task_Stop();                           /* KEY1 运行时 = 急停 */
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
        default: Task_Stop(); break;   /* 异常任务号，安全停车 */
    }
}

/* ── 灰度事件标志位（5ms ISR 置位，任务消费清零）── */
/* 检测器内部 LOCKOUT 保证每次入/离线只触发一次，不会反复置位 */
volatile uint8_t s_gray_enter_fired = 0U;   /* 入线事件 */
volatile uint8_t s_gray_exit_fired  = 0U;   /* 出线事件 */ 

/* ══════════════════════════════════════════════════════════════════════
 * Task_1 — 直走遇线停车
 *
 * 流程：
 *   启动 → 速度环 25 + 偏航角保持 0° 直走
 *        → GrayDetect_EnterLine 触发（白底→黑线）
 *        → 停车 + PID 复位 + 声光提示
 * ══════════════════════════════════════════════════════════════════════ */
void Task_1(void)
{
    static uint8_t s_state  = 0U;   /* 0=待初始化, 1=运行中 */
    static uint8_t s_cool   = 0U;   /* 入线检测冷却计数器     */
    static uint8_t s_my_gen = 0U;

    /* ── 感知任务被重启 ── */
    if (s_my_gen != s_gen)
    {
        s_my_gen = s_gen;
        s_state  = 0U;
        s_cool   = 0U;
    }

    /* ── 首次进入：设置速度与偏航角目标（仅一次）── */
    if (s_state == 0U)
    {
        LED_Control(LED1, LED_LOW);
        PID_EncoderSpeed_Set(&speed_loop, 20.0f, 150.0f, 0.0f, 15.0f);
        YawPid_InitStraight();                       /* 直走专用：Out_max=300 */
        YawPid_Set(0.3f, 0.03f, 0.0f, 0.0f);        /* 温和偏航角：kp 0.3 */
        PID_Reset(&yaw_pid);
        s_cool  = 5U;                                /* 100ms 冷却：防上电噪点误触发 */
        s_state = 1U;
    }

    /* ── 每 20ms：速度环 + 偏航角环融合输出 ── */
    Drive_YawSpeed();

    /* ── 遇线停车（冷却期内不检测）── */
    if (s_cool > 0U)
    {
        s_cool--;
    }
    else if (s_gray_enter_fired != 0U)
    {
        s_gray_enter_fired = 0U;
        s_gray_exit_fired  = 0U;               /* 互斥：同时清零对方 */
        Task_Stop();                       /* 停车 + 复位全部 PID */
        LED_Control(LED1, LED_HIGH);
        Buzzer_Alert(200);                 /* 声光提示              */
        s_state = 0U;                      /* 下次启动重新初始化    */
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * Task_2 — 循迹一圈（A→B→C→D→A）
 *
 * 地图：
 *   A ───(白底,yaw=0°)───→ B ───(黑曲线,灰度巡线)───→ C
 *   │                                                    │
 *   └───(黑曲线,灰度巡线)─── D ←───(白底,yaw=180°)───┘
 *
 * 状态流转：
 *   S_A_B   yaw PID(0°)  + 速度环，遇线 → S_B_C
 *   S_B_C   灰度巡线      + 速度环，离线 → S_C_D
 *   S_C_D   yaw PID(180°)+ 速度环，遇线 → S_D_A
 *   S_D_A   灰度巡线      + 速度环，离线 → S_DONE（一圈完成）
 *
 * 速度环全程运行（25 编码器 ticks/20ms）。
 * ══════════════════════════════════════════════════════════════════════ */

/* ── 任务 2 内部状态枚举 ── */
typedef enum {
    T2_INIT = 0U,   /* 首次进入，设 PID 参数                         */
    T2_A_B,         /* A→B：白底直走，偏航角 0°                       */
    T2_B_C,         /* B→C：黑线巡线，灰度方向环                       */
    T2_C_D,         /* C→D：白底直走，偏航角 180°（反向）              */
    T2_D_A,         /* D→A：黑线巡线，灰度方向环                       */
    T2_DONE          /* 一圈完成，停车 + 声光提示                      */
} Task2_State_t;

void Task_2(void)
{
    static Task2_State_t s_state = T2_INIT;
    static Task2_State_t s_prev  = T2_INIT;
    static uint8_t       s_cool  = 0U;   /* 入线检测冷却计数器（每 20ms -1） */
    static uint8_t       s_my_gen = 0U;   /* 上次运行的代次，与 s_gen 比较感知重启 */

    /* ── 感知任务被重启（KEY1 重新按下）── */
    if (s_my_gen != s_gen)
    {
        s_my_gen = s_gen;
        s_state  = T2_INIT;
        s_prev   = T2_INIT;
        s_cool   = 0U;
    }

    /* ── 检测状态切换（用于"首次进入"逻辑）── */
    uint8_t state_entered = (s_state != s_prev);
    s_prev = s_state;

    /* ── 首次进入：设置速度环、方向环、偏航角环的 PID 参数 ── */
    if (s_state == T2_INIT)
    {
        PID_EncoderSpeed_Set(&speed_loop, 20.0f, 150.0f, 0.0f, 15.0f);
        Set_PID(&yaw_pid,        0.3f, 0.03f, 0.0f);       /* 偏航角环参数      */
        PID_SetOutputLimit(&yaw_pid, 400);                 /* 直走舵量限 ±400    */
        Set_PID(&direction_pid,  1.0f, 0.002f, 0.01f);      /* 灰度方向环参数    */
        PID_Reset(&yaw_pid);
        PID_Reset(&direction_pid);
        s_task2_pos = 1U;   /* A 点 */
        s_state = T2_A_B;
    }

    /* ── 状态机 ── */
    switch (s_state)
    {

    /* ══════════════════════════════════════════════════════════════════
     * A→B：白底直走，偏航角保持 0°，遇黑线进入巡线
     * ══════════════════════════════════════════════════════════════════ */
    case T2_A_B:
        if (state_entered)
        {
            s_task2_pos = 1U;                       /* A 点 */
            s_cool      = 10U;                       /* 200ms 冷却：防边界噪点误触发 */
            YawPid_SetTarget(0.0f);                 /* 目标 0°      */
            PID_Reset(&yaw_pid);
            PID_Reset(&speed_loop.left);             /* 清速度环积分：防上一段残留差速 */
            PID_Reset(&speed_loop.right);
        }
        Drive_YawSpeed();   /* 偏航角(0°) + 速度环 */

        if (s_cool > 0U)
        {
            s_cool--;
        }
        else if (s_gray_enter_fired != 0U)
        {
            s_gray_enter_fired = 0U;
            s_gray_exit_fired  = 0U;               /* 互斥 */
            Buzzer_Alert(200);                      /* 点 B：声光提示        */
            s_state = T2_B_C;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * B→C：黑线巡线，灰度方向环 + 速度环，离线后切换偏航角
     * ══════════════════════════════════════════════════════════════════ */
    case T2_B_C:
        if (state_entered)
        {
            s_task2_pos = 2U;                       /* B 点 */
            PID_Reset(&direction_pid);              /* 巡线前清方向环积分    */
            PID_Reset(&speed_loop.left);             /* 清速度环积分：A→B yaw 差速残留 */
            PID_Reset(&speed_loop.right);
        }
        LineFollow_Output();   /* 速度环 + g_steer */

        if (s_gray_exit_fired != 0U)
        {
            s_gray_exit_fired  = 0U;
            s_gray_enter_fired = 0U;               /* 互斥 */
            Buzzer_Alert(200);                      /* 点 C：声光提示        */
            s_state = T2_C_D;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * C→D：白底直走，偏航角保持 180°，遇线进入第二次巡线
     * ══════════════════════════════════════════════════════════════════ */
    case T2_C_D:
        if (state_entered)
        {
            s_task2_pos = 3U;                       /* C 点 */
            s_cool      = 10U;                       /* 200ms 冷却：刚从黑线出来，防边界噪点 */
            YawPid_SetTarget(172.0f);               /* 目标 170°     */
            PID_Reset(&yaw_pid);
            PID_Reset(&speed_loop.left);             /* ★ 关键：B→C 巡线曲线段左右轮差速导致 */
            PID_Reset(&speed_loop.right);            /*         速度环积分不对称，必须清零     */
        }
        Drive_YawSpeed();   /* 偏航角(180°) + 速度环 */

        if (s_cool > 0U)
        {
            s_cool--;
        }
        else if (s_gray_enter_fired != 0U)
        {
            s_gray_enter_fired = 0U;
            s_gray_exit_fired  = 0U;               /* 互斥 */
            Buzzer_Alert(200);                      /* 点 D：声光提示        */
            s_state = T2_D_A;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * D→A：黑线巡线，灰度方向环 + 速度环，离线后一圈完成
     * ══════════════════════════════════════════════════════════════════ */
    case T2_D_A:
        if (state_entered)
        {
            s_task2_pos = 4U;                       /* D 点 */
            PID_Reset(&direction_pid);              /* 巡线前清方向环积分    */
            PID_Reset(&speed_loop.left);             /* 清速度环积分：C→D yaw 差速残留 */
            PID_Reset(&speed_loop.right);
        }
        LineFollow_Output();   /* 速度环 + g_steer */

        if (s_gray_exit_fired != 0U)
        {
            s_gray_exit_fired  = 0U;
            s_gray_enter_fired = 0U;               /* 互斥 */
            Buzzer_Alert(200);                      /* 点 A：声光提示        */
            s_state = T2_DONE;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * 一圈完成：停车 + 复位 + 声光提示
     * ══════════════════════════════════════════════════════════════════ */
    case T2_DONE:
        s_task2_pos = 5U;                       /* 完成 */
        Task_Stop();
        Buzzer_Alert(500);
        s_state = T2_INIT;  /* 下次启动重新开始 */
        break;

    default:
        s_state = T2_INIT;
        break;
    }
}

/* ── 任务 3/4 可调参数 ── */
#define TASK4_TARGET_LAPS     4U        /* 任务 4 目标圈数                          */

/* ── 开环旋转参数（固定占空比 + 定时，不依赖陀螺仪）── */
/* 旋转方向：TASK3_DRIVE_YAW_A/B 为负 = 逆时针 = 左转（左轮后、右轮前）    */
#define TASK3_SPIN_DUTY_A   300     /* A 点旋转占空比（左转为正）              */
#define TASK3_SPIN_TICKS_A  45U     /* A 点旋转 tick 数（×20ms）               */
#define TASK3_SPIN_DUTY_B   -300     /* B 点旋转占空比                          */
#define TASK3_SPIN_TICKS_B  45U    /* B 点旋转 tick 数（×20ms）               */

/* ── A→C / B→D 直走偏航角 PID 目标角度（度）── */
#define TASK3_DRIVE_YAW_A   (-36.8f)  /* A→C 直走偏航角保持目标               */
#define TASK3_DRIVE_YAW_B   (-149.0f) /* B→D 直走偏航角保持目标               */

/* ══════════════════════════════════════════════════════════════════════
 * Task_3 — 交叉循迹一圈（A→C→B→D→A）
 *
 * 地图：
 *   A ──(白底,yaw=TASK3_DRIVE_YAW_A)──→ C ───(黑曲线,灰度巡线)───→ B
 *   │                                                             │
 *   └──(黑曲线,灰度巡线)─── A ←───(白底,yaw=TASK3_DRIVE_YAW_B)─── D ┘
 *
 * 状态流转：
 *   T3_A_C_SPIN   A 点：开环差速（TASK3_SPIN_DUTY/A + TASK3_SPIN_TICKS/A）
 *   T3_A_C_GO     yaw PID(TASK3_DRIVE_YAW_A) + 速度环，遇线 → C 点
 *   T3_C_B        灰度巡线      + 速度环，离线 → B 点
 *   T3_B_D_SPIN   B 点：开环差速（TASK3_SPIN_DUTY/B + TASK3_SPIN_TICKS/B）
 *   T3_B_D_GO     yaw PID(TASK3_DRIVE_YAW_B) + 速度环，遇线 → D 点
 *   T3_D_A        灰度巡线      + 速度环，离线 → A 点（一圈完成）
 *
 * 速度环全程运行（15 编码器 ticks/20ms）。
 * 旋转阶段：开环定时差速，不读陀螺仪，不经过速度环。
 * ══════════════════════════════════════════════════════════════════════ */

/* ── 任务 3 内部状态枚举 ── */
typedef enum {
    T3_INIT = 0U,       /* 首次进入，设 PID 参数                         */
    T3_A_C_SPIN,        /* A 点：原地旋转到 -45°                         */
    T3_A_C_GO,          /* A→C：偏航角 -45° 白底直走，遇线 → C           */
    T3_C_B,             /* C→B：黑线巡线，离线 → B                       */
    T3_B_D_SPIN,        /* B 点：原地旋转到 -135°                        */
    T3_B_D_GO,          /* B→D：偏航角 -135° 白底直走，遇线 → D          */
    T3_D_A,             /* D→A：黑线巡线，离线 → A                       */
    T3_DONE             /* 一圈完成，停车 + 声光提示                      */
} Task3_State_t;

/*
 * Task34_Run — Task_3 / Task_4 共享状态机（A→C→B→D→A 轨迹）
 *
 * @param max_laps  目标圈数：Task_3 = 1 圈，Task_4 = TASK4_TARGET_LAPS 圈
 *
 * 每圈的最后一步（D→A巡线离线到达A点）自动判断：
 *   圈数未满 → 回到 A_C_SPIN 开始下一圈
 *   圈数已满 → T3_DONE 停车
 */
static void Task34_Run(uint8_t max_laps)
{
    static Task3_State_t s_state    = T3_INIT;
    static Task3_State_t s_prev     = T3_INIT;
    static uint8_t       s_cool     = 0U;   /* 入线检测冷却计数器（每 20ms -1）       */
    static uint8_t       s_spin_tick = 0U;  /* 开环旋转 tick 计数（每 20ms -1）          */
    static uint8_t       s_lap      = 0U;   /* 已完成圈数                               */
    static uint8_t       s_my_gen   = 0U;

    /* ── 感知任务被重启（KEY1 重新按下）── */
    if (s_my_gen != s_gen)
    {
        s_my_gen = s_gen;
        s_state  = T3_INIT;
        s_prev   = T3_INIT;
        s_cool   = 0U;
        s_spin_tick = 0U;
        s_lap       = 0U;
    }

    /* ── 检测状态切换（用于"首次进入"逻辑）── */
    uint8_t state_entered = (s_state != s_prev);
    s_prev = s_state;

    /* ── 首次进入：设置速度环 + 方向环 PID 参数（偏航角环在各阶段独立配置）── */
    if (s_state == T3_INIT)
    {
        PID_EncoderSpeed_Set(&speed_loop, 20.0f, 150.0f, 0.0f, 18.0f);
        Set_PID(&direction_pid,  1.0f, 0.002f, 0.01f);      /* 灰度方向环参数    */
        PID_Reset(&direction_pid);
        s_task3_pos = 1U;   /* A 点 */
        s_state = T3_A_C_SPIN;
    }

    /* ── 状态机 ── */
    switch (s_state)
    {
    /* ══════════════════════════════════════════════════════════════════
     * A 点：原地旋转到 -45°（纯差速，不经过速度环）
     * ══════════════════════════════════════════════════════════════════ */
    case T3_A_C_SPIN:
        if (state_entered)
        {
            s_task3_pos = 1U;
            s_spin_tick = TASK3_SPIN_TICKS_A;
        }
        /* 左转：左轮后、右轮前 → 逆时针旋转 */
        TB6612_SetSpeed(-(int16_t)TASK3_SPIN_DUTY_A, (int16_t)TASK3_SPIN_DUTY_A);
        if (--s_spin_tick == 0U)
        {
            TB6612_SetSpeed(0, 0);  /* 旋转结束先停车 */
            s_state = T3_A_C_GO;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * A→C：白底直走，偏航角保持 -45°，遇黑线到达 C 点
     * ══════════════════════════════════════════════════════════════════ */
    case T3_A_C_GO:
        if (state_entered)
        {
            s_task3_pos = 1U;                       /* A→C 途中 */
            s_cool      = 10U;                       /* 200ms 冷却：防旋转后边界噪点 */
            YawPid_Set(0.3f, 0.03f, 0.0f, TASK3_DRIVE_YAW_A);  /* 直走温和偏航角 + 保持目标 */
            PID_SetOutputLimit(&yaw_pid, 400);       /* 直走舵量限 ±400             */
            PID_Reset(&yaw_pid);
            PID_Reset(&speed_loop.left);             /* ★ 清速度环积分：旋转后重新起步 */
            PID_Reset(&speed_loop.right);
        }
        Drive_YawSpeed();   /* 偏航角(-45°) + 速度环 */

        if (s_cool > 0U)
        {
            s_cool--;
        }
        else if (s_gray_enter_fired != 0U)
        {
            s_gray_enter_fired = 0U;
            s_gray_exit_fired  = 0U;               /* 互斥 */
            Buzzer_Alert(200);                      /* 点 C：声光提示        */
            s_state = T3_C_B;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * C→B：黑线巡线，灰度方向环 + 速度环，离线到达 B 点
     * ══════════════════════════════════════════════════════════════════ */
    case T3_C_B:
        if (state_entered)
        {
            s_task3_pos = 2U;                       /* C→B 巡线中 */
            PID_Reset(&direction_pid);              /* 巡线前清方向环积分    */
            PID_Reset(&speed_loop.left);             /* ★ 清速度环积分        */
            PID_Reset(&speed_loop.right);
        }
        LineFollow_Output();   /* 速度环 + g_steer */

        if (s_gray_exit_fired != 0U)
        {
            s_gray_exit_fired  = 0U;
            s_gray_enter_fired = 0U;               /* 互斥 */
            Buzzer_Alert(200);                      /* 点 B：声光提示        */
            s_state = T3_B_D_SPIN;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * B 点：原地旋转到 -135°（纯差速，不经过速度环）
     * ══════════════════════════════════════════════════════════════════ */
    case T3_B_D_SPIN:
        if (state_entered)
        {
            s_task3_pos = 3U;
            s_spin_tick = TASK3_SPIN_TICKS_B;
        }
        /* 左转：左轮后、右轮前 → 逆时针旋转 */
        TB6612_SetSpeed(-(int16_t)TASK3_SPIN_DUTY_B, (int16_t)TASK3_SPIN_DUTY_B);
        if (--s_spin_tick == 0U)
        {
            TB6612_SetSpeed(0, 0);  /* 旋转结束先停车 */
            s_state = T3_B_D_GO;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * B→D：白底直走，偏航角保持 -135°，遇黑线到达 D 点
     * ══════════════════════════════════════════════════════════════════ */
    case T3_B_D_GO:
        if (state_entered)
        {
            s_task3_pos = 3U;                       /* B→D 途中 */
            s_cool      = 10U;                       /* 200ms 冷却 */
            YawPid_Set(0.3f, 0.03f, 0.0f, TASK3_DRIVE_YAW_B); /* 直走温和偏航角 + 保持目标 */
            PID_SetOutputLimit(&yaw_pid, 400);
            PID_Reset(&yaw_pid);
            PID_Reset(&speed_loop.left);             /* ★ 清速度环积分 */
            PID_Reset(&speed_loop.right);
        }
        Drive_YawSpeed();   /* 偏航角(-135°) + 速度环 */

        if (s_cool > 0U)
        {
            s_cool--;
        }
        else if (s_gray_enter_fired != 0U)
        {
            s_gray_enter_fired = 0U;
            s_gray_exit_fired  = 0U;               /* 互斥 */
            Buzzer_Alert(200);                      /* 点 D：声光提示        */
            s_state = T3_D_A;
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * D→A：黑线巡线，灰度方向环 + 速度环，离线到达 A 点，一圈完成
     * ══════════════════════════════════════════════════════════════════ */
    case T3_D_A:
        if (state_entered)
        {
            s_task3_pos = 4U;                       /* D→A 巡线中 */
            PID_Reset(&direction_pid);              /* 巡线前清方向环积分    */
            PID_Reset(&speed_loop.left);             /* ★ 清速度环积分        */
            PID_Reset(&speed_loop.right);
        }
        LineFollow_Output();   /* 速度环 + g_steer */

        if (s_gray_exit_fired != 0U)
        {
            s_gray_exit_fired  = 0U;
            s_gray_enter_fired = 0U;               /* 互斥 */
            Buzzer_Alert(200);                      /* 点 A：声光提示        */

            s_lap++;                                /* 完成一圈              */
            if (s_lap >= max_laps)
            {
                s_state = T3_DONE;                   /* 圈数够 → 停车        */
            }
            else
            {
                s_state = T3_A_C_SPIN;               /* 圈数不够 → 继续下一圈 */
            }
        }
        break;

    /* ══════════════════════════════════════════════════════════════════
     * 一圈完成：停车 + 复位 + 声光提示
     * ══════════════════════════════════════════════════════════════════ */
    case T3_DONE:
        s_task3_pos = 5U;                       /* 完成 */
        Task_Stop();
        Buzzer_Alert(500);
        s_state = T3_INIT;  /* 下次启动重新开始 */
        break;

    default:
        s_state = T3_INIT;
        break;
    }
}

/*
 * Task_3 — 交叉循迹 1 圈（A→C→B→D→A），到 A 停车。
 */
void Task_3(void)
{
    Task34_Run(1U);
}

/* ══════════════════════════════════════════════════════════════════════
 * Task_4 — 同 Task_3 轨迹，自动行驶 TASK4_TARGET_LAPS 圈后停车
 *
 * 地图、状态流转与 Task_3 完全一致，仅在到达 A 点时判断圈数：
 *   圈数 <  TASK4_TARGET_LAPS → 回到 A_C_SPIN 继续下一圈
 *   圈数 >= TASK4_TARGET_LAPS → T3_DONE 停车
 * ══════════════════════════════════════════════════════════════════════ */
void Task_4(void)
{
    Task34_Run(TASK4_TARGET_LAPS);
}
