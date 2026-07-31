#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#include "PID/PID.h"
#include "Filter/Filter.h"
#include "gray_adc.h"   /* GrayADC_Sensor_t 类型（g_graySensor 保留兼容） */
#include "LED.h"     /* LED_Id_t for Buzzer_Light */

#ifdef __cplusplus
extern "C" {
#endif

/* 速度环 */
extern PID_EncoderSpeed_t speed_loop;

/* 方向环（灰度循线位置 PID） */
extern PID_TypeDef direction_pid;

extern GrayADC_Sensor_t g_graySensor;

/* 任务选择（KEY.c 定义，KEY2 循环 1→4） */
extern volatile uint8_t s_task_select;

/* ── PID 对象 ── */
extern PID_TypeDef yaw_pid;             /* 偏航角位置环 PID */
extern PID_TypeDef ball_pid_pos;        /* 正目标 PID（X>0，短力臂侧） */
extern PID_TypeDef ball_pid_neg;        /* 负目标 PID（X<0，长力臂侧） */

/* PID 初始化（速度环 + 方向环 + 偏航角环） */
void PID_Control_Init(void);
void YawPid_Init(void);                 /* 偏航角 PID 默认初始化（Out_max=API_MOTOR_MAX_DUTY）  */
void YawPid_InitStraight(void);         /* 直走专用：小死区 + 低输出上限 ±1200    */
void YawPid_Set(float kp, float ki, float kd, float target_deg);  /* 四合一：PID 参数 + 目标角度 */
void YawPid_SetTarget(float degrees);   /* 单独设置目标偏航角（度）             */
int32_t YawPid_Calc(float yaw_degrees); /* 计算偏航角 PID 输出（度，直接传 jy->yaw）*/
void YawTest_Control(void);             /* 偏航角单独测试（纯差速，绕过速度环）  */

/*
 * 小球位置环 — 摄像头 X 坐标 → 双向独立 PID → Stepmotor_SetAngle
 *
 * BallPid_Init:     初始化 ball_pid_pos / ball_pid_neg 两个实例
 * BallPid_SetTarget:同时设两个 PID 的目标，由 Move_Control 自动选 active
 * BallPid_Calc:     给定当前 X 坐标，内部缩放 100× 送整数 PID
 * Ball_Move_Control:自动选 pos/neg PID → 计算 → Stepmotor_SetAngle
 * BallTest_Control: 单独测试版，含丢球保护 + 限幅
 */

/* 步进电机输出角度限幅 */
#define BALL_ANGLE_MAX     20.0f    /* 电机正方向最大角度（°） */
#define BALL_ANGLE_MIN    -20.0f    /* 电机负方向最大角度（°） */

void BallPid_Init(void);
void BallPid_SetTarget(float target_x);
int32_t BallPid_Calc(float ball_x);
void Ball_Move_Control(void);
void BallTest_Control(void);             /* 小球方向环单独测试（摄像头数据源）  */

/*
 * 方向环控制（TIMG0 ISR 中 5ms 调用一次）。
 *
 * 内部自动：
 *   2. 方向 PID 计算（整数）→ 更新全局 steer 变量
 */
void Direction_Control(void);

/*
 * 电机输出限幅到 API_MOTOR_MAX_DUTY。
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
void PID_Speed_Control(void);

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
