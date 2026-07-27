#ifndef __API_MOTOR_H
#define __API_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * API_Motor — 统一电机控制接口（编译期静态分发）
 *
 * 通过 MOTOR_DRIVER 宏选择底层驱动，零运行时开销：
 *   #define MOTOR_DRIVER MOTOR_DRIVER_AT4950  ← 当前
 *   #define MOTOR_DRIVER MOTOR_DRIVER_TB6612  ← 回退方案
 *
 * 编译器在 -O2 下会直接把调用内联为底层函数，和直接调用无区别。
 */

#define MOTOR_DRIVER_AT4950  1
#define MOTOR_DRIVER_TB6612  2

#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER  MOTOR_DRIVER_AT4950
#endif

/* ── 当前驱动对应的占空比上限（统一宏，app 层无需知道底层驱动）── */
#if (MOTOR_DRIVER == MOTOR_DRIVER_AT4950)
#include "AT4950.h"
#define API_MOTOR_MAX_DUTY  AT4950_MAX_DUTY    /* 4000 — PD1 80MHz, 20kHz */
#elif (MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
#include "TB6612.h"
#define API_MOTOR_MAX_DUTY  TB6612_MAX_DUTY    /* 2000 — PD0 40MHz, 20kHz */
#endif

/*
 * API_Motor_Init — 初始化当前选定的电机驱动
 *
 * AT4950: 上电刹车（IN1=IN2=100%）
 * TB6612: 方向脚全部拉低
 */
void API_Motor_Init(void);

/*
 * API_Motor_SetSpeed — 设置 A/B 两路电机速度
 *
 *   speed > 0 → 正转，绝对值 = 占空比
 *   speed < 0 → 反转，绝对值 = 占空比
 *   speed = 0 → 刹车 / 停止
 *
 * AT4950: 快衰减单极性 PWM，含方向反转死区保护
 * TB6612: GPIO 方向 + 统一 PWM
 */
void API_Motor_SetSpeed(int16_t speedA, int16_t speedB);

#ifdef __cplusplus
}
#endif

#endif /* __API_MOTOR_H__ */
