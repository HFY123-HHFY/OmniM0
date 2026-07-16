#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#include "PID/PID.h"
#include "Filter/Filter.h"
#include "gray_adc.h"
#include "LED.h"     /* LED_Id_t for LED_TurnNb_Start */

#ifdef __cplusplus
extern "C" {
#endif

/* 速度环 */
extern PID_EncoderSpeed_t speed_loop;

/* 方向环（灰度循线位置 PID） */
extern PID_TypeDef direction_pid;

/* 灰度传感器实例（main.c 定义，Control 层引用） */
extern GrayADC_Sensor_t g_graySensor;

/* 目标圈数（KEY.c 定义，KEY4 设置，Control 层读取） */
extern volatile uint8_t s_target_laps;

/* PID 初始化（速度环 + 方向环） */
void PID_Control_Init(void);

/*
 * 方向环控制（TIMG0 ISR 中 5ms 调用一次）。
 *
 * 内部自动：
 *   1. 读 g_graySensor 最新线位置
 *   2. 方向 PID 计算（整数）→ 更新全局 steer 变量
 */
void Direction_Control(void);

/*
 * 电机输出限幅到 TB6612_MAX_DUTY (±400)。
 */
void MotorOutput_Clamp(int16_t *left, int16_t *right);

/*
 * 循线融合输出（TIMG0 ISR 中 20ms 调用一次）。
 * 速度环整数 PID → 差速输出到电机，唯一写 TB6612 的入口。
 */
void LineFollow_Output(int32_t actual_left, int32_t actual_right);

/*
 * 速度环独立控制（纯速度模式，不使用方向环 steer）。
 */
void PID_Speed_Control(int32_t actual_left, int32_t actual_right);

/*
 * 方向环单独测试（纯差速转向，绕过速度环）。
 */
void Direction_Test_Control(void);

/*
 * Control_Run — 循线主控（TIMG0 ISR 中 20ms 调用一次）。
 * 内部：按键启停、路口检测、转弯状态机、速度+方向融合。
 * 所有 PID 运算均为整数，适合 ISR 上下文（M0+ 无 FPU）。
 */
void Control_Run(int32_t actual_left, int32_t actual_right);

/*
 * Control_IsTurning — 返回 1 表示正在转弯。
 * 主循环方向环用这个判断是否跳过 Direction_Control。
 */
uint8_t Control_IsTurning(void);

/*
 * Control_GetTargetLaps — 返回目标圈数 (1-5)。
 */
uint8_t Control_GetTargetLaps(void);

/*
 * Control_GetIntersectionCount — 返回已过路口数。
 */
uint8_t Control_GetIntersectionCount(void);

#ifdef __cplusplus
}
#endif

/* ══════════════════════════════════════════════════════════════════════
 * 非阻塞蜂鸣器 — 基于 g_sys_tick_ms，替代阻塞式 LED_Turn
 *
 * 用法：
 *   LED_TurnNb_Start(Buzzer1, 200);               // 开蜂鸣器，启动 200ms 延时
 *   LED_TurnNb_Task();                             // 主循环中调用，超时后自动关闭
 *
 * 支持 N 个独立通道（Buzzer1 / LED1 / LED2 / LED3）。
 * ══════════════════════════════════════════════════════════════════════ */
void LED_TurnNb_Start(LED_Id_t id, uint32_t periodMs);
void LED_TurnNb_Task(void);

/* ══════════════════════════════════════════════════════════════════════
 * 任务链调度 — KEY1 启动/停止，KEY2 循环选择任务 (1-4)
 *
 * Task_Run 在 TIMG0 ISR 20ms 插槽调用（与 Control_Run 同位置）。
 * 启动瞬间锁存任务号，运行中 KEY2 不影响当前任务。
 * ══════════════════════════════════════════════════════════════════════ */
void    Task_Run(int32_t actual_left, int32_t actual_right);
void    Task_Stop(void);           /* 停车 + PID 复位，任务内部可调用   */
uint8_t Task_IsRunning(void);      /* 1 = 运行中                        */
uint8_t Task_GetSelect(void);      /* KEY2 当前选中任务号 (1-4)         */
uint8_t Task_GetActive(void);      /* 正在运行的任务号，待机时为 0      */

/* 任务 */
void Task_1(void);
void Task_2(void);
void Task_3(void);
void Task_4(void);

#endif /* CONTROL_H */
