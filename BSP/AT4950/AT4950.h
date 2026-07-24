#ifndef __AT4950_H
#define __AT4950_H

#include <stdint.h>
#include "pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AT4950 双路 H 桥电机驱动 — 快衰减 + 单极性 PWM
 *
 * 控制逻辑（A/B 两路相同）：
 *   正转：IN1 = PWM,  IN2 = 0
 *   反转：IN1 = 0,    IN2 = PWM
 *   刹车：IN1 = 1,    IN2 = 1  （100% 占空比 → H 桥同侧上管导通短路制动）
 *
 * 电机A：AIN1 → TIM1 CH1 (B15=TIMG7-CH0), AIN2 → TIM2 CH1 (B7=TIMG6-CH1)
 * 电机B：BIN1 → TIM3 CH1 (B13=TIMA0-CH3), BIN2 → TIM3 CH2 (B9=TIMA0-CH1)
 * 全部挂 PD1 BUSCLK 80MHz, period=4000, 20kHz
 */

/* ── 电机A PWM 映射 ── */
#define AT4950_AIN1_PWM_TIM   (API_PWM_TIM1)
#define AT4950_AIN1_PWM_CH    (API_PWM_CH1)

#define AT4950_AIN2_PWM_TIM   (API_PWM_TIM2)
#define AT4950_AIN2_PWM_CH    (API_PWM_CH1)

/* ── 电机B PWM 映射 ── */
#define AT4950_BIN1_PWM_TIM   (API_PWM_TIM3)
#define AT4950_BIN1_PWM_CH    (API_PWM_CH1)

#define AT4950_BIN2_PWM_TIM   (API_PWM_TIM3)
#define AT4950_BIN2_PWM_CH    (API_PWM_CH2)

/* ── 占空比参数 ── */
#define AT4950_MAX_DUTY       (4000U)   /* PD1 80MHz, period=4000, 20kHz      */
#define AT4950_FULL_DUTY      (4000U)   /* 100% = 刹车（SetCCR 内部钳位-1）   */
#define AT4950_DEAD_TIME_MS   (2U)      /* 方向切换死区：刹车→释放的稳定时间  */

/*
 * AT4950_Init — 初始化：上电立刻刹车，防止误触发
 *
 * 必须在 PWM 初始化（API_PWM_Init）之后调用。
 */
void AT4950_Init(void);

/*
 * AT4950_SetSpeed — 统一设置 A/B 两路电机速度
 *
 *   speedA > 0 → A正转,  占空比 = speedA（钳位 0~MAX_DUTY）
 *   speedA < 0 → A反转,  占空比 = -speedA
 *   speedA = 0 → A刹车
 *
 *   speedB 同理。
 *
 * 方向切换（正转↔反转）时自动插入 AT4950_DEAD_TIME_MS 的刹车死区，
 * 防止 H 桥瞬间反向直通烧毁 MOS 管。
 */
void AT4950_SetSpeed(int16_t speedA, int16_t speedB);

#ifdef __cplusplus
}
#endif

#endif /* __AT4950_H__ */
