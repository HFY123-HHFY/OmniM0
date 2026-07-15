#include "G3507_pwm.h"
#include "pwm.h"

#include "G3507_hw_config.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_timera.h"
#include "ti/driverlib/dl_timerg.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"

/*
 * G3507 PWM 底层实现。
 *
 * 支持两种定时器类型：
 *   - TIMA0/TIMA1：高级定时器（DL_TimerA_* API）
 *   - TIMG8：      通用定时器（DL_TimerG_* API）
 *
 * 注意：DL_TimerA_* 和 DL_TimerG_* 都是 DL_Timer_* 统一 API 的宏别名，
 *       但为代码可读性，按定时器类型显式区分。
 */

typedef struct
{
	GPTIMER_Regs *regs;
} G3507_PWM_Map_t;

#define G3507_PWM_POWER_STARTUP_DELAY 16U

static void G3507_PWM_PowerStartupDelay(void)
{
	volatile uint32_t cycles;
	for (cycles = 0U; cycles < G3507_PWM_POWER_STARTUP_DELAY; ++cycles)
	{
		__NOP();
	}
}

/*
 * 获取定时器实例寄存器基址。
 * 返回 GPTIMER_Regs* 统一指针（TIMA 和 TIMG 共用 xDL_Timer 统一 API）。
 */
static G3507_PWM_Map_t G3507_PWM_GetMap(uint8_t coreTimId)
{
	G3507_PWM_Map_t map;
	map.regs = 0;

	switch (coreTimId)
	{
	case API_PWM_CORE_TIMA0:
		map.regs = (GPTIMER_Regs *)TIMA0;
		break;
	case API_PWM_CORE_TIMA1:
		map.regs = (GPTIMER_Regs *)TIMA1;
		break;
	case API_PWM_CORE_TIMG8:
		map.regs = TIMG8;   /* TIMG8 本身就是 GPTIMER_Regs* */
		break;
	default:
		break;
	}

	return map;
}

/* 仅用于判断定时器类型（TIMA vs TIMG），控制电源上电/复位分支 */
static uint8_t G3507_PWM_IsTimerG(uint8_t coreTimId)
{
	return (coreTimId == API_PWM_CORE_TIMG8) ? 1U : 0U;
}

/*
 * 根据 psc+1 推算 DL_Timer 的 divideRatio 和 prescale。
 * TIMA 和 TIMG 共用相同的时钟分频逻辑。
 */
static uint8_t G3507_PWM_GetClockDivider(uint32_t divisor,
                                         DL_TIMER_CLOCK_DIVIDE *divideRatio,
                                         uint8_t *prescale)
{
	static const uint8_t s_dividers[] = {8U, 4U, 2U, 1U};
	uint8_t i;

	if ((divideRatio == 0) || (prescale == 0) || (divisor == 0U))
	{
		return 0U;
	}

	for (i = 0U; i < (uint8_t)(sizeof(s_dividers) / sizeof(s_dividers[0])); ++i)
	{
		uint32_t ratio          = s_dividers[i];
		uint32_t scaledPrescale;

		if ((divisor % ratio) != 0U) { continue; }

		scaledPrescale = divisor / ratio;
		if ((scaledPrescale == 0U) || (scaledPrescale > 256U)) { continue; }

		switch (ratio)
		{
		case 8U: *divideRatio = DL_TIMER_CLOCK_DIVIDE_8; break;
		case 4U: *divideRatio = DL_TIMER_CLOCK_DIVIDE_4; break;
		case 2U: *divideRatio = DL_TIMER_CLOCK_DIVIDE_2; break;
		default: *divideRatio = DL_TIMER_CLOCK_DIVIDE_1; break;
		}

		*prescale = (uint8_t)(scaledPrescale - 1U);
		return 1U;
	}

	return 0U;
}

/* ══════════════════════════════════════════════════════════════════════
 * G3507_PWM_ConfigPin — 配置指定定时器通道的 PWM 引脚 IOMUX
 * ══════════════════════════════════════════════════════════════════════ */
void G3507_PWM_ConfigPin(uint8_t coreTimId, uint8_t coreChannel)
{
	/* ── TIMG8 引脚（当前板级：PB7=CCP1, PB15=CCP0）── */
	if (coreTimId == API_PWM_CORE_TIMG8)
	{
		if (coreChannel == API_PWM_CORE_CCP0)
		{
			DL_GPIO_initPeripheralOutputFunction(G3507_PWM_CH0_IOMUX,
			                                     G3507_PWM_CH0_FUNC);
			DL_GPIO_enableOutput(G3507_PWM_CH0_PORT,
			                     G3507_PWM_CH0_PIN);
		}
		else if (coreChannel == API_PWM_CORE_CCP1)
		{
			DL_GPIO_initPeripheralOutputFunction(G3507_PWM_CH1_IOMUX,
			                                     G3507_PWM_CH1_FUNC);
			DL_GPIO_enableOutput(G3507_PWM_CH1_PORT,
			                     G3507_PWM_CH1_PIN);
		}
		return;
	}

}

/* ══════════════════════════════════════════════════════════════════════
 * G3507_PWM_InitTimer — 初始化 PWM 定时器（TIMA / TIMG 通用）
 * ══════════════════════════════════════════════════════════════════════ */
void G3507_PWM_InitTimer(uint8_t coreTimId, uint16_t arr, uint16_t psc)
{
	G3507_PWM_Map_t map;
	uint8_t         isTG;
	uint32_t        pwmPeriod;
	uint32_t        divisor;
	DL_Timer_ClockConfig clockConfig;    /* 统一 API 结构体 */

	map = G3507_PWM_GetMap(coreTimId);
	if (map.regs == 0) { return; }

	isTG = G3507_PWM_IsTimerG(coreTimId);

	/* ── 电源上电 / 复位 ── */
	if (isTG)
	{
		DL_TimerG_reset(map.regs);
		if (!DL_TimerG_isPowerEnabled(map.regs))
		{
			DL_TimerG_enablePower(map.regs);
			G3507_PWM_PowerStartupDelay();
			while (!DL_TimerG_isPowerEnabled(map.regs)) { }
		}
	}
	else
	{
		DL_TimerA_reset(map.regs);
		if (!DL_TimerA_isPowerEnabled(map.regs))
		{
			DL_TimerA_enablePower(map.regs);
			G3507_PWM_PowerStartupDelay();
			while (!DL_TimerA_isPowerEnabled(map.regs)) { }
		}
	}

	/* ── 时钟配置（TIMA / TIMG 共用相同的 ClockConfig 结构）── */
	divisor = (uint32_t)psc + 1UL;
	if (G3507_PWM_GetClockDivider(divisor,
	                              &clockConfig.divideRatio,
	                              &clockConfig.prescale) == 0U) { return; }

	clockConfig.clockSel = DL_TIMER_CLOCK_BUSCLK;
	if (isTG) { DL_TimerG_setClockConfig(map.regs, &clockConfig); }
	else      { DL_TimerA_setClockConfig(map.regs, &clockConfig); }

	/* ── PWM 周期 ── */
	pwmPeriod = (uint32_t)arr + 1UL;
	if (isTG) { DL_TimerG_setLoadValue(map.regs, pwmPeriod - 1UL); }
	else      { DL_TimerA_setLoadValue(map.regs, pwmPeriod - 1UL); }

	/* ── CCP 通道配置（TIMA 和 TIMG 完全相同的寄存器布局）── */

#define PWM_CCP_CFG(prefix) do {                                       \
	/* 输出极性：高电平有效（CNT>CC = high, CNT≤CC = low）        */  \
	(prefix##_setCaptureCompareAction)(map.regs,                        \
	    (DL_TIMER_CC_LACT_CCP_HIGH | DL_TIMER_CC_CDACT_CCP_LOW),       \
	    DL_TIMER_CC_0_INDEX);                                          \
	(prefix##_setCaptureCompareAction)(map.regs,                        \
	    (DL_TIMER_CC_LACT_CCP_HIGH | DL_TIMER_CC_CDACT_CCP_LOW),       \
	    DL_TIMER_CC_1_INDEX);                                          \
	/* 比较模式 */                                                     \
	(prefix##_setCaptureCompareCtl)(map.regs,                           \
	    DL_TIMER_CC_MODE_COMPARE, 0U, DL_TIMER_CC_0_INDEX);            \
	(prefix##_setCaptureCompareCtl)(map.regs,                           \
	    DL_TIMER_CC_MODE_COMPARE, 0U, DL_TIMER_CC_1_INDEX);            \
	/* 输入直通（不反相，选 CCPx） */                                  \
	(prefix##_setCaptureCompareInput)(map.regs,                         \
	    DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_IN_SEL_CCPX,       \
	    DL_TIMER_CC_0_INDEX);                                          \
	(prefix##_setCaptureCompareInput)(map.regs,                         \
	    DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_IN_SEL_CCPX,       \
	    DL_TIMER_CC_1_INDEX);                                          \
	/* 输出控制：初始低电平，不反相，使用功能值 */                     \
	(prefix##_setCaptureCompareOutCtl)(map.regs,                        \
	    DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED, \
	    DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMER_CC_0_INDEX);            \
	(prefix##_setCaptureCompareOutCtl)(map.regs,                        \
	    DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED, \
	    DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMER_CC_1_INDEX);            \
	/* 立即更新 */                                                     \
	(prefix##_setCaptCompUpdateMethod)(map.regs,                        \
	    DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_0_INDEX);     \
	(prefix##_setCaptCompUpdateMethod)(map.regs,                        \
	    DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_1_INDEX);     \
} while (0)

	if (isTG)
	{
		PWM_CCP_CFG(DL_TimerG);
		DL_TimerG_setCounterRepeatMode(map.regs, DL_TIMER_REPEAT_MODE_ENABLED);
		DL_TimerG_setCounterValueAfterEnable(map.regs, DL_TIMER_COUNT_AFTER_EN_LOAD_VAL);
		DL_TimerG_setCounterControl(map.regs,
		                            DL_TIMER_CZC_CCCTL0_ZCOND,
		                            DL_TIMER_CAC_CCCTL0_ACOND,
		                            DL_TIMER_CLC_CCCTL0_LCOND);
		DL_TimerG_setCCPDirection(map.regs, (DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT));
		DL_TimerG_setCCPOutputDisabled(map.regs,
		                               DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
		                               DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);

		DL_TimerG_setTimerCount(map.regs, 0U);
		DL_TimerG_setCaptureCompareValue(map.regs, 0U, DL_TIMER_CC_0_INDEX);
		DL_TimerG_setCaptureCompareValue(map.regs, 0U, DL_TIMER_CC_1_INDEX);
		DL_TimerG_enableClock(map.regs);
		DL_TimerG_startCounter(map.regs);
	}
	else
	{
		PWM_CCP_CFG(DL_TimerA);
		DL_TimerA_setCounterRepeatMode(map.regs, DL_TIMER_REPEAT_MODE_ENABLED);
		DL_TimerA_setCounterValueAfterEnable(map.regs, DL_TIMER_COUNT_AFTER_EN_LOAD_VAL);
		DL_TimerA_setCounterControl(map.regs,
		                            DL_TIMER_CZC_CCCTL0_ZCOND,
		                            DL_TIMER_CAC_CCCTL0_ACOND,
		                            DL_TIMER_CLC_CCCTL0_LCOND);
		DL_TimerA_setCCPDirection(map.regs, (DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT));
		DL_TimerA_setCCPOutputDisabled(map.regs,
		                               DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
		                               DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);

		DL_TimerA_setTimerCount(map.regs, 0U);
		DL_TimerA_setCaptureCompareValue(map.regs, 0U, DL_TIMER_CC_0_INDEX);
		DL_TimerA_setCaptureCompareValue(map.regs, 0U, DL_TIMER_CC_1_INDEX);
		DL_TimerA_enableClock(map.regs);
		DL_TimerA_startCounter(map.regs);
	}

#undef PWM_CCP_CFG
}

/* ══════════════════════════════════════════════════════════════════════
 * G3507_PWM_SetCCR — 设置 PWM 占空比
 *
 * Edge-aligned down-count 模式：
 *   compareValue = period - duty
 *   当 CNT > CC 时输出高，CNT ≤ CC 时输出低。
 *
 * TIMA 和 TIMG 使用相同的寄存器布局，直接用 DL_Timer 统一 API。
 * ══════════════════════════════════════════════════════════════════════ */
void G3507_PWM_SetCCR(uint8_t coreTimId, uint8_t coreChannel, uint16_t ccr)
{
	G3507_PWM_Map_t map;
	DL_TIMER_CC_INDEX ccIndex;
	uint32_t period;
	uint32_t duty;
	uint32_t compareValue;

	map = G3507_PWM_GetMap(coreTimId);
	if (map.regs == 0) { return; }

	if (coreChannel == API_PWM_CORE_CCP0)
	{
		ccIndex = DL_TIMER_CC_0_INDEX;
	}
	else if (coreChannel == API_PWM_CORE_CCP1)
	{
		ccIndex = DL_TIMER_CC_1_INDEX;
	}
	else
	{
		return;
	}

	period = DL_Timer_getLoadValue(map.regs) + 1UL;
	duty   = (uint32_t)ccr;
	if (duty > period) { duty = period; }

	/* Edge-aligned down-count: CCP high = period - CC, 所以 CC = period - duty */
	compareValue = period - duty;
	DL_Timer_setCaptureCompareValue(map.regs, compareValue, ccIndex);
}
