#include "AT4950.h"
#include "pwm.h"
#include "Delay.h"

/*
 * AT4950 双路 H 桥电机驱动 — 快衰减 + 单极性 PWM
 *
 * 状态机设计：
 *   ┌─────────┐  speed!=0   ┌──────────┐
 *   │  刹车    │ ──────────→ │  正常运行  │
 *   │ IN1=IN2=1│ ←────────── │ IN1/2=PWM │
 *   └─────────┘  speed=0     └──────────┘
 *        ↑                        │
 *        │   方向符号变化           │
 *        └────────────────────────┘
 *   （自动插入死区 Delay_ms(DEAD_TIME_MS)）
 *
 * 安全要点：
 *   1. 方向切换必经刹车中间态，避免 H 桥瞬间直通
 *   2. 上电初始化时刹车，防止 PWM 未就绪时误触发
 *   3. 刹车使用 100% 占空比（IN=1）→ 同侧上管导通短路制动
 *   4. 死区期间电流通过 FET 体二极管续流，不产生高压尖峰
 */

/* ── 取符号：正→1, 负→-1, 零→0 ── */
static int8_t AT4950_Sign(int16_t v)
{
	if (v > 0) { return 1; }
	if (v < 0) { return -1; }
	return 0;
}

/* ── |speed| → 占空比，钳位到 MAX_DUTY ── */
static uint16_t AT4950_AbsToDuty(int16_t value)
{
	uint32_t duty;

	if (value < 0)
	{
		duty = (uint32_t)(-value);
	}
	else
	{
		duty = (uint32_t)value;
	}

	if (duty > AT4950_MAX_DUTY)
	{
		duty = AT4950_MAX_DUTY;
	}

	return (uint16_t)duty;
}

/* ── 单路刹车 ── */
static void AT4950_BrakeA(void)
{
	API_PWM_Setcom(AT4950_AIN1_PWM_TIM, AT4950_AIN1_PWM_CH, AT4950_FULL_DUTY);
	API_PWM_Setcom(AT4950_AIN2_PWM_TIM, AT4950_AIN2_PWM_CH, AT4950_FULL_DUTY);
}

static void AT4950_BrakeB(void)
{
	API_PWM_Setcom(AT4950_BIN1_PWM_TIM, AT4950_BIN1_PWM_CH, AT4950_FULL_DUTY);
	API_PWM_Setcom(AT4950_BIN2_PWM_TIM, AT4950_BIN2_PWM_CH, AT4950_FULL_DUTY);
}

/* ── 单路驱动（直通，不做补偿——死区由 PID 积分项自然克服）── */
static void AT4950_DriveA(int8_t sign, uint16_t duty)
{
	if (sign > 0)
	{
		/* 正转：IN1=PWM, IN2=0 */
		API_PWM_Setcom(AT4950_AIN1_PWM_TIM, AT4950_AIN1_PWM_CH, duty);
		API_PWM_Setcom(AT4950_AIN2_PWM_TIM, AT4950_AIN2_PWM_CH, 0U);
	}
	else /* sign < 0 */
	{
		/* 反转：IN1=0, IN2=PWM */
		API_PWM_Setcom(AT4950_AIN1_PWM_TIM, AT4950_AIN1_PWM_CH, 0U);
		API_PWM_Setcom(AT4950_AIN2_PWM_TIM, AT4950_AIN2_PWM_CH, duty);
	}
}

static void AT4950_DriveB(int8_t sign, uint16_t duty)
{
	if (sign > 0)
	{
		API_PWM_Setcom(AT4950_BIN1_PWM_TIM, AT4950_BIN1_PWM_CH, duty);
		API_PWM_Setcom(AT4950_BIN2_PWM_TIM, AT4950_BIN2_PWM_CH, 0U);
	}
	else /* sign < 0 */
	{
		API_PWM_Setcom(AT4950_BIN1_PWM_TIM, AT4950_BIN1_PWM_CH, 0U);
		API_PWM_Setcom(AT4950_BIN2_PWM_TIM, AT4950_BIN2_PWM_CH, duty);
	}
}

/* ══════════════════════════════════════════════════════════════════════
 * AT4950_Init — 上电立刻刹车，防止误触发
 * ══════════════════════════════════════════════════════════════════════ */
void AT4950_Init(void)
{
	AT4950_BrakeA();
	AT4950_BrakeB();
}

/* ══════════════════════════════════════════════════════════════════════
 * AT4950_SetSpeed — 统一设置 A/B 两路电机速度和方向
 *
 *  速度语义：
 *    > 0 → 正转，绝对值 = 占空比
 *    < 0 → 反转，绝对值 = 占空比
 *    = 0 → 刹车（IN1=IN2=1，短路制动）
 *
 *  状态切换保护：
 *    方向变化时自动插入刹车死区（Delay_ms），给 MOSFET
 *    和电机绕组足够的电流衰减时间，再切换到新方向。
 * ══════════════════════════════════════════════════════════════════════ */
void AT4950_SetSpeed(int16_t speedA, int16_t speedB)
{
	static int8_t prevSignA = 0;
	static int8_t prevSignB = 0;

	int8_t   signA, signB;
	uint16_t dutyA, dutyB;
	uint8_t  needBrakeA = 0U, needBrakeB = 0U;

	signA = AT4950_Sign(speedA);
	signB = AT4950_Sign(speedB);
	dutyA = AT4950_AbsToDuty(speedA);
	dutyB = AT4950_AbsToDuty(speedB);

	/*
	 * 检测方向反转（正→负 或 负→正）→ 需插入刹车死区
	 * 同方向调速 / 停车 / 从停车起步 → 不需要死区
	 */
	if ((prevSignA * signA) < 0) { needBrakeA = 1U; }
	if ((prevSignB * signB) < 0) { needBrakeB = 1U; }

	/* ── 阶段 1：方向反转死区刹车 ── */
	if (needBrakeA) { AT4950_BrakeA(); }
	if (needBrakeB) { AT4950_BrakeB(); }

	if (needBrakeA || needBrakeB)
	{
		/*
		 * 死区等待：MOSFET 关断 + 电机电流衰减。
		 * 2ms 在 20kHz PWM 下约 40 个周期，足以让 H 桥稳定。
		 */
		Delay_ms(AT4950_DEAD_TIME_MS);
	}

	/* ── 阶段 2：目标状态 ── */

	/* A 相 */
	if (signA == 0)
	{
		AT4950_BrakeA();    /* 停车 = 持续刹车 */
	}
	else
	{
		AT4950_DriveA(signA, dutyA);
	}

	/* B 相 */
	if (signB == 0)
	{
		AT4950_BrakeB();
	}
	else
	{
		AT4950_DriveB(signB, dutyB);
	}

	prevSignA = signA;
	prevSignB = signB;
}
