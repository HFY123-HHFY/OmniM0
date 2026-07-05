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

/* PID 初始化（速度环 + 方向环） */
void PID_Control_Init(void);

/*
 * 方向环控制（TIM1 中断中 5ms 调用一次）。
 *
 * 内部自动：
 *   1. 读 g_graySensor 最新线位置
 *   2. 方向 PID 计算 → 更新全局 steer 变量
 *
 * 注意：本函数只算不写电机，steer 由 LineFollow_Output 在 20ms 融合输出。
 */
void Direction_Control(void);

/*
 * 电机输出限幅到 TB6612_MAX_DUTY (±400)。
 * 所有写电机的函数统一走这里。
 */
void MotorOutput_Clamp(int16_t *left, int16_t *right);

/*
 * 循线融合输出（TIM2 中断中 20ms 调用一次）— 正式循线用。
 *
 * 速度环 PID → 基础速度，叠加方向环 steer，差速输出到电机。
 */
void LineFollow_Output(float actual_left, float actual_right);

/*
 * 速度环控制函数（纯速度模式，不使用方向环 steer）。
 */
void PID_Speed_Control(float actual_left, float actual_right);

/*
 * 方向环单独测试（纯差速转向，绕过速度环）。
 * 测试方向 PID 时，在 Control_Task 20ms 回调里替换 LineFollow_Output。
 */
void Direction_Test_Control(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
