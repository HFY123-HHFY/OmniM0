#include "Control.h"

#include "My_Usart/My_Usart.h"          /* usart_printf */
#include "Control_Task/Control_Task.h"   /* NonBlockDelay_t */
#include "pwm.h"
#include "TB6612.h"
#include "KEY.h"
#include "LED.h"

/* =========================================================================
 * PID 对象
 * ========================================================================= */
/* 速度环 */
PID_EncoderSpeed_t speed_loop;

/* 方向环（灰度循线位置 PID） */
PID_TypeDef direction_pid;

/* 灰度传感器实例（Control.h 中 extern，供 Control_Task / main 引用） */
GrayADC_Sensor_t g_graySensor;

/*
 * PID_Control_Init — 速度环 + 方向环 结构初始化。
 *
 * 所有参数使用 Q16.16 整数或自然单位，适配 M0+ 无 FPU。
 * kp / ki / kd / target 等参数请到 main.c 里设置。
 */
void PID_Control_Init(void)
{
    /* ── 速度环结构（左右轮独立 PID）── */
    PID_EncoderSpeed_Init(&speed_loop);
    PID_SetSampleTime(&speed_loop.left,  20);   /* dt = 20ms  */
    PID_SetSampleTime(&speed_loop.right, 20);

    /* ── 方向环结构 ── */
    PID_Init(&direction_pid);
    PID_Init_WithLimit(&direction_pid, 100, 1500); /* I_out 最多 ±100, Out_max=1500（占空比域，2000 标度） */
    PID_SetSampleTime(&direction_pid, 5);        /* dt = 5ms                    */
    PID_SetDeadband(&direction_pid, 60);         /* 死区 60（线位置 mm×100）     */
    PID_SetIntegralSeparation(&direction_pid, 3000); /* 积分分离阈值 3000        */
}

/* 方向环输出暂存（Direction_Control 写，LineFollow_Output 读） */
static int32_t g_steer = 0;

/* =========================================================================
 * MotorOutput_Clamp — 电机输出限幅到 TB6612_MAX_DUTY (±2000)
 * ========================================================================= */
void MotorOutput_Clamp(int16_t *left, int16_t *right)
{
    if (*left  >  (int16_t)TB6612_MAX_DUTY) *left  =  (int16_t)TB6612_MAX_DUTY;
    if (*left  < -(int16_t)TB6612_MAX_DUTY) *left  = -(int16_t)TB6612_MAX_DUTY;
    if (*right >  (int16_t)TB6612_MAX_DUTY) *right =  (int16_t)TB6612_MAX_DUTY;
    if (*right < -(int16_t)TB6612_MAX_DUTY) *right = -(int16_t)TB6612_MAX_DUTY;
}

/* =========================================================================
 * Direction_Control — 方向环（TIMG0 ISR 5ms）
 *
 * 灰度线位置 → 方向 PID → g_steer（整数，全部 Q16.16 计算在 PID 库内完成）
 * ========================================================================= */
void Direction_Control(void)
{
    int32_t pos    = GrayADC_LinePosition(&g_graySensor);
    int32_t center = (int32_t)(7U * GRAY_ADC_SENSOR_SPACING_MM * 100U / 2U);

    if (pos < 0)
    {
        return; /* 传感器未校准/丢线，保持上一拍 steer */
    }

    PID_SetTarget(&direction_pid, center);
    g_steer = -PID_Calc(&direction_pid, pos);  /* PID_Calc 返回 int32_t */
}

/* =========================================================================
 * LineFollow_Output — 速度环 + 方向环融合输出（TIMG0 ISR 20ms）
 *
 * steer>0 → 左轮加速、右轮减速 → 车右转
 * steer<0 → 左轮减速、右轮加速 → 车左转
 * ========================================================================= */
void LineFollow_Output(int32_t actual_left, int32_t actual_right)
{
    int32_t out_left  = 0;
    int32_t out_right = 0;
    int16_t left, right;

    /* ── 1. 速度环（整数 PID）── */
    PID_EncoderSpeed_Control(&speed_loop, actual_left, actual_right,
                             &out_left, &out_right);

    /* ── 2. 融合方向环 steer ── */
    // left  = (int16_t)out_left  + (int16_t)g_steer;
    // right = (int16_t)out_right - (int16_t)g_steer;

    left  = (int16_t)out_left;
    right = (int16_t)out_right;

    /* ── 3. 限幅 + 写电机 ── */
    MotorOutput_Clamp(&left, &right);
    TB6612_SetSpeed(left, right);
}

/*
 * 速度环独立控制（纯速度模式）。
 */
void PID_Speed_Control(int32_t actual_left, int32_t actual_right)
{
    int32_t out_left  = 0;
    int32_t out_right = 0;
    int16_t left, right;

    PID_EncoderSpeed_Control(&speed_loop, actual_left, actual_right,
                             &out_left, &out_right);

    left  = (int16_t)out_left;
    right = (int16_t)out_right;

    MotorOutput_Clamp(&left, &right);
    TB6612_SetSpeed(left, right);
}

/*
 * 方向环单独测试 — 纯差速转向。
 */
void Direction_Test_Control(void)
{
    int16_t left  = (int16_t)g_steer;
    int16_t right = -(int16_t)g_steer;

    MotorOutput_Clamp(&left, &right);
    TB6612_SetSpeed(left, right);
}

/* ══════════════════════════════════════════════════════════════════════
 * Control_Run — 所有循线控制逻辑的入口（TIMG0 ISR 20ms）
 * ══════════════════════════════════════════════════════════════════════ */

/* ── 转弯参数 ── */
#define TURN_DELAY_MS   30U
#define TURN_PIVOT_MS   250U

#define TURN_DELAY_TICK  (TURN_DELAY_MS  / 20U)
#define TURN_PIVOT_TICK  (TURN_PIVOT_MS / 20U)

#define INTERSECTIONS_PER_LAP  4U

static uint8_t  s_running            = 0U;
static uint8_t  s_delaying           = 0U;
static uint8_t  s_turning            = 0U;
static uint16_t s_delay_tick         = 0U;
static uint16_t s_turn_tick          = 0U;
static uint8_t  s_intersection_count = 0U;
static uint8_t  s_need_white         = 0U;

uint8_t Control_IsTurning(void)          { return s_turning; }
uint8_t Control_GetTargetLaps(void)      { return s_target_laps; }
uint8_t Control_GetIntersectionCount(void){ return s_intersection_count; }

void Control_Run(int32_t actual_left, int32_t actual_right)
{
    /* ── 按键 ── */
    if (Key == 1U && s_running == 0U)
    {
        Key                  = 0U;
        s_running            = 1U;
        s_delaying           = 0U;
        s_turning            = 0U;
        s_delay_tick         = 0U;
        s_turn_tick          = 0U;
        s_intersection_count = 0U;
        s_need_white         = 0U;
        PID_Reset(&direction_pid);
        PID_Reset(&speed_loop.left);
        PID_Reset(&speed_loop.right);
    }
    else if (Key == 2U)
    {
        Key          = 0U;
        s_running    = 0U;
        s_delaying   = 0U;
        s_turning    = 0U;
        s_delay_tick = 0U;
        s_turn_tick  = 0U;
        TB6612_SetSpeed(0, 0);
        PID_Reset(&direction_pid);
        PID_Reset(&speed_loop.left);
        PID_Reset(&speed_loop.right);
    }

    if (s_running == 0U) return;

    /* ── 转弯 / 等待 / 直走 ── */
    if (s_turning)
    {
        if (s_turn_tick < TURN_PIVOT_TICK)
        {
            TB6612_SetSpeed(-675, 750);   /* 差速转弯（2000 标度，等效旧 -135/150） */
        }
        else
        {
            s_turning    = 0U;
            s_need_white = 1U;
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);

            if (s_intersection_count >= s_target_laps * INTERSECTIONS_PER_LAP)
            {
                s_running = 0U;
                TB6612_SetSpeed(0, 0);
            }
        }
        s_turn_tick++;
    }
    else if (s_delaying)
    {
        if (s_delay_tick >= TURN_DELAY_TICK)
        {
            s_delaying  = 0U;
            s_turning   = 1U;
            s_turn_tick = 0U;
        }
        s_delay_tick++;
        LineFollow_Output(actual_left, actual_right);
    }
    else
    {
        if (s_need_white != 0U)
        {
            if (g_graySensor.digital_bits[0] == 1U &&
                g_graySensor.digital_bits[1] == 1U)
            {
                s_need_white = 0U;
            }
        }
        else if (g_graySensor.digital_bits[0] == 0U &&
                 g_graySensor.digital_bits[1] == 0U)
        {
            s_intersection_count++;
            s_delaying   = 1U;
            s_delay_tick = 0U;
            LED_Control(LED2, LED_HIGH);
        }
        else
        {
            LED_Control(LED2, LED_LOW);
        }
        LineFollow_Output(actual_left, actual_right);
    }
}

/*
 * 非阻塞声光提示
 */

#define LED_TURNNB_MAX  4U    /* Buzzer1 / LED1 / LED2 / LED3 */

/*
 * 每个 LED/Buzzer 通道一份独立状态，互不干扰。
 * busy=0 空闲，busy=1 正在计时等待关闭。
 */
typedef struct {
    NonBlockDelay_t delay;
    uint8_t         busy;
} LED_TurnNb_Ch_t;

static LED_TurnNb_Ch_t s_ledNbCh[LED_TURNNB_MAX];

/*
 * LED_TurnNb_Start — 启动一次非阻塞高低闪烁。
 *
 * @param id       LED 逻辑编号（LED1 / LED2 / LED3 / Buzzer1）
 * @param periodMs 高电平持续时长（ms）
 *
 * 立即打开 LED，启动毫秒延时，函数立即返回不阻塞。
 * 重复 Start() 同一个通道会覆盖上一次未到期的延时。
 */
void LED_TurnNb_Start(LED_Id_t id, uint32_t periodMs)
{
    uint8_t idx = (uint8_t)id;

    if (periodMs == 0U || idx >= LED_TURNNB_MAX)
    {
        return;
    }

    LED_Control(id, LED_HIGH);                       /* 立刻开    */
    NonBlockDelay_Start(&s_ledNbCh[idx].delay, (uint16_t)periodMs);
    s_ledNbCh[idx].busy = 1U;
}

/*
 * LED_TurnNb_Task — 非阻塞蜂鸣器调度函数。
 *
 * 须放在主循环 while(1) 中每次调用。
 * 扫描所有通道，超时则自动关闭对应的 LED/Buzzer。
 * 无活跃通道时仅 ~1µs（一次 for 循环比较）。
 */
void LED_TurnNb_Task(void)
{
    uint8_t i;

    for (i = 0U; i < LED_TURNNB_MAX; ++i)
    {
        if (s_ledNbCh[i].busy == 0U)
        {
            continue;
        }

        if (NonBlockDelay_IsDone(&s_ledNbCh[i].delay))
        {
            LED_Control((LED_Id_t)i, LED_LOW);       /* 时间到，关 */
            s_ledNbCh[i].busy = 0U;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * 任务链调度 — KEY1 启动/停止，KEY2 循环选择任务 (1-4)
 *
 * s_task_select 由 KEY.c 维护（KEY2 循环 1→4，同 KEY3 设圈数模式）。
 * 启动时把 s_task_select 锁存到 s_task_active，运行中按 KEY2 不会切换任务。
 * Task_Run 在 TIMG0 ISR 20ms 插槽调用（与 Control_Run 同位置）。
 * ══════════════════════════════════════════════════════════════════════ */

static uint8_t s_task_running = 0U;   /* 0 = 待机，1 = 运行中           */
static uint8_t s_task_active  = 0U;   /* 启动瞬间锁存的任务号 (1-4)     */

uint8_t Task_IsRunning(void)  { return s_task_running; }
uint8_t Task_GetSelect(void)  { return s_task_select; }
uint8_t Task_GetActive(void)  { return s_task_active; }

/*
 * Task_Stop — 停止当前任务：停车 + 复位所有 PID。
 * 任务函数内部达成结束条件时也可直接调用。
 */
void Task_Stop(void)
{
    s_task_running = 0U;
    s_task_active  = 0U;
    TB6612_SetSpeed(0, 0);
    PID_Reset(&direction_pid);
    PID_Reset(&speed_loop.left);
    PID_Reset(&speed_loop.right);
}

/*
 * Task_Run — 任务链入口（TIMG0 ISR 20ms 调用一次）。
 *
 * KEY1 消费模式（读后清零，防全局变量持久化导致重复触发）：
 *   待机时按下 → 锁存 s_task_select 并启动；
 *   运行时按下 → 急停（Task_Stop）。
 */
void Task_Run(int32_t actual_left, int32_t actual_right)
{
    (void)actual_left;
    (void)actual_right;

    /* ── 按键 ── */
    if (Key == 1U)
    {
        Key = 0U;
        if (s_task_running == 0U)
        {
            s_task_running = 1U;
            s_task_active  = s_task_select;
            PID_Reset(&direction_pid);
            PID_Reset(&speed_loop.left);
            PID_Reset(&speed_loop.right);
        }
        else
        {
            Task_Stop();
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

/* 任务1 */
void Task_1(void)
{

}

/* 任务2 */
void Task_2(void)
{

}

/* 任务3 */
void Task_3(void)
{

}

/* 任务4 */
void Task_4(void)
{

}
