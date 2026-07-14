#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#include "PID/PID.h"
#include "Filter/Filter.h"
#include "gray_adc.h"

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

#endif /* CONTROL_H */
