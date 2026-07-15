#include "Control.h"

#include "My_Usart/My_Usart.h"          /* usart_printf */
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
    PID_Init_WithLimit(&direction_pid, 20, 300); /* I_out 最多 ±20, Out_max=180 */
    PID_SetSampleTime(&direction_pid, 5);        /* dt = 5ms                    */
    PID_SetDeadband(&direction_pid, 60);         /* 死区 60（线位置 mm×100）     */
    PID_SetIntegralSeparation(&direction_pid, 3000); /* 积分分离阈值 3000        */
}

/* 方向环输出暂存（Direction_Control 写，LineFollow_Output 读） */
static int32_t g_steer = 0;

/* =========================================================================
 * MotorOutput_Clamp — 电机输出限幅到 TB6612_MAX_DUTY (±400)
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
            TB6612_SetSpeed(-135, 150);
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
