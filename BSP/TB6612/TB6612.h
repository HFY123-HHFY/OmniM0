#ifndef __TB6612_H
#define __TB6612_H

#include <stdint.h>

/* TB6612 默认使用的 PWM 定时器与通道映射（A/B 相各自独立定时器）。
 * PCBV3.0：
 *   PWMA ← PB15 (TIMG7_CCP0, PD1 80MHz) → API_PWM_TIM1, CH1
 *   PWMB ← PB7  (TIMG6_CCP1, PD1 80MHz) → API_PWM_TIM2, CH1
 */
#define TB6612_PWM_TIM_A      (API_PWM_TIM1)
#define TB6612_PWM_CH_A       (API_PWM_CH1)
#define TB6612_PWM_TIM_B      (API_PWM_TIM2)
#define TB6612_PWM_CH_B       (API_PWM_CH1)

/*
* TB6612 占空比上限（= PWM ARR+1，即满占空比对应 4000）
* 80MHz / 1 / 4000 = 20kHz，每步 0.025%
* 映射到编码器上：占空比2000、编码器：30; 占空比4000、编码器：60
*/
#define TB6612_MAX_DUTY       (4000U) /* 20kHz @ 4000 步，每步 0.025% */

#define TB6612_WRITE(port, pin, level) API_GPIO_Write((port), (pin), (uint8_t)((level) ? 1U : 0U))

/* 方向控制快捷宏：AIN1/AIN2/BIN1/BIN2。 */
#define AIN1_OUT(x) TB6612_WRITE(s_tb6612ConfigTable[0].ain1Port, s_tb6612ConfigTable[0].ain1Pin, (x))
#define AIN2_OUT(x) TB6612_WRITE(s_tb6612ConfigTable[0].ain2Port, s_tb6612ConfigTable[0].ain2Pin, (x))
#define BIN1_OUT(x) TB6612_WRITE(s_tb6612ConfigTable[0].bin1Port, s_tb6612ConfigTable[0].bin1Pin, (x))
#define BIN2_OUT(x) TB6612_WRITE(s_tb6612ConfigTable[0].bin2Port, s_tb6612ConfigTable[0].bin2Pin, (x))

/* TB6612 方向脚资源映射，仅包含 AIN/BIN 四个方向控制脚。 */
typedef struct
{
	void *ain1Port;
	uint32_t ain1Pin;
	void *ain2Port;
	uint32_t ain2Pin;
	void *bin1Port;
	uint32_t bin1Pin;
	void *bin2Port;
	uint32_t bin2Pin;
} TB6612_Config_t;

/* 注册 TB6612 配置表。 */
void TB6612_Register(const TB6612_Config_t *configTable, uint8_t count);
/* 初始化 TB6612 方向脚。 */
void TB6612_Init(void);
/* 设置 A/B 两路电机速度（正负表示方向，绝对值表示占空比）。 */
void TB6612_SetSpeed(int16_t speedA, int16_t speedB);

#endif /* __TB6612_H__ */
