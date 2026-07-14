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
 * 方向环控制（主循环中 5ms 调用一次，由 tasks.dir_5ms.flag 触发）。
 *
 * 内部自动：
 *   1. 读 g_graySensor 最新线位置
 *   2. 方向 PID 计算 → 更新全局 steer 变量
 *
 * 注意：本函数只算不写电机，steer 由 LineFollow_Output 在 20ms 融合输出。
 *       严禁在 ISR 中调用（含浮点运算，M0+ 无硬件 FPU）。
 */
void Direction_Control(void);

/*
 * 电机输出限幅到 TB6612_MAX_DUTY (±400)。
 * 所有写电机的函数统一走这里。
 */
void MotorOutput_Clamp(int16_t *left, int16_t *right);

/*
 * 循线融合输出（主循环中 20ms 调用一次，由 tasks.encoder_20ms.flag 触发）。
 *
 * 速度环 PID → 基础速度，叠加方向环 steer，差速输出到电机。
 * 唯一写 TB6612 的入口。
 */
void LineFollow_Output(float actual_left, float actual_right);

/*
 * 速度环控制函数（纯速度模式，不使用方向环 steer）。
 */
void PID_Speed_Control(float actual_left, float actual_right);

/*
 * 方向环单独测试（纯差速转向，绕过速度环）。
 * 测试方向 PID 时，在主循环 20ms 分支里替换 LineFollow_Output。
 */
void Direction_Test_Control(void);

/*
 * Control_Run — 循线主控（主循环中 20ms 调用一次，由 tasks.encoder_20ms.flag 触发）。
 *
 * 内部包含：按键启停、路口检测、转弯状态机、速度+方向融合输出。
 * 主循环调度器只需调这一个函数，所有控制逻辑收敛在这里。
 */
void Control_Run(float actual_left, float actual_right);

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
