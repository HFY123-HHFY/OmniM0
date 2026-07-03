#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#include "PID/PID.h"
#include "Filter/Filter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ±2000dps 量程下 MPU6050 陀螺灵敏度：16.4 LSB/(deg/s) */
#define GYRO_SENS_2000DPS (16.4f)

/* PID 输出加载到电机前的总限幅。 */
#define MOTOR_MIX_LIMIT (2047.0f)

/* 目标姿态/高度。 */
extern float Target_Pitch;
extern float Target_Roll;
extern float Target_Yaw;

/* 外环 PID：角度。 */
extern PID_TypeDef pid_pitch;
extern PID_TypeDef pid_roll;
extern PID_TypeDef pid_yaw;

/* 内环 PID：角速度。 */
extern PID_TypeDef pid_rate_pitch;
extern PID_TypeDef pid_rate_roll;
extern PID_TypeDef pid_rate_yaw;

/* 速度环 */
extern PID_EncoderSpeed_t speed_loop;

/*
 * 控制初始化：
 */
void PID_Contorl_Init(void);
void PID_Speed_Init(void);

/*
 * Pitch/Roll 串级 PID 控制。
*/
void PID_Pitch_Roll_Combined(float actual_pitch, float actual_roll);

/* 
 * 速度环控制函数 
*/
void PID_Speed_Control(float actual_left, float actual_right);

/*
 * 陀螺零偏校准（上电后调用一次，飞行器必须静止）。
 * samples        : 采样点数（建议 1000，约 5s）
 * gravity_ref_out: 输出重力参考值 (aacz 均值)，可为 NULL
 * 返回 1 成功，0 超时失败。
 */
uint8_t GyroBias_Calibrate(uint16_t samples, float *gravity_ref_out);

/* 查询校准是否完成。 */
uint8_t GyroBias_IsReady(void);

/*
 * 预留电机加载接口。
 * 当前默认实现仅做占位，方便后续接真实电机混控。
 */
void Motor_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
