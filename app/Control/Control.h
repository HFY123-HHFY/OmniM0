#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#include "PID/PID.h"
#include "Filter/Filter.h"
#include "gray_adc.h"
#include "LED.h"     /* LED_Id_t for Buzzer_Light */

#ifdef __cplusplus
extern "C" {
#endif

/* 速度环 */
extern PID_EncoderSpeed_t speed_loop;

/* 方向环（灰度循线位置 PID） */
extern PID_TypeDef direction_pid;

/* 灰度传感器实例（main.c 定义，Control 层引用） */
extern GrayADC_Sensor_t g_graySensor;

/* 任务选择（KEY.c 定义，KEY2 循环 1→4） */
extern volatile uint8_t s_task_select;

/* ── PID 对象 ── */
extern PID_TypeDef yaw_pid;             /* 偏航角位置环 PID */

/* PID 初始化（速度环 + 方向环 + 偏航角环） */
void PID_Control_Init(void);
void YawPid_Init(void);                 /* 偏航角 PID 默认初始化（Out_max=1500）  */
void YawPid_InitStraight(void);         /* 直走专用：小死区 + 低输出上限 ±300    */
void YawPid_Set(float kp, float ki, float kd, float target_deg);  /* 四合一：PID 参数 + 目标角度 */
void YawPid_SetTarget(float degrees);   /* 单独设置目标偏航角（度）             */
int32_t YawPid_Calc(float yaw_degrees); /* 计算偏航角 PID 输出（度，直接传 jy->yaw）*/
void YawTest_Control(void);             /* 偏航角单独测试（纯差速，绕过速度环）  */

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
 * 速度环 + 灰度方向环融合输出（TIMG0 ISR 20ms）。
 * 内部直读 Encoder1/2_Speed，融合 g_steer。
 */
void LineFollow_Output(void);

/*
 * 速度环 + 偏航角环融合输出（TIMG0 ISR 20ms）。
 * 内部直读 Encoder1/2_Speed + JY61P_GetYawFiltered()，融合 yaw_steer。
 */
void Drive_YawSpeed(void);

/*
 * 速度环独立控制（纯速度模式，不使用方向环 steer）。
 */
void PID_Speed_Control(int32_t actual_left, int32_t actual_right);

/*
 * 方向环单独测试（纯差速转向，绕过速度环）。
 */
void Direction_Test_Control(void);

/*
 * 内部：按键启停、路口检测、转弯状态机、速度+方向融合。
 * 所有 PID 运算均为整数，适合 ISR 上下文（M0+ 无 FPU）。
 */

/*
 * 主循环方向环用这个判断是否跳过 Direction_Control。
 */

#ifdef __cplusplus
}
#endif


#endif /* CONTROL_H */
