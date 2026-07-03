#include "sys.h"
#include "G3507_sys.h"

uint8_t SYS_EXTI_GetLineIndex(uint32_t pin)
{
	uint8_t index;

	for (index = 0U; index < 16U; ++index)
	{
		if (pin == (uint32_t)(1UL << index))
		{
			return index;
		}
	}

	return 0xFFU;
}

void SYS_Init(void)
{
	G3507_SYS_Init();
}

uint32_t SYS_EXTI_GetIrqn(void *port, uint32_t pin)
{
	if ((port == 0) || (pin == 0U))
	{
		return SYS_EXTI_INVALID_IRQN;
	}

	/* G3507 GPIOA/GPIOB 使用端口中断，支持 B24-B27 等高位引脚。
	 * 仅需根据端口选择 IRQn，不再限制 lineIndex <= 15。
	 */
	if (port == GPIOA)
	{
		return (uint32_t)GPIOA_INT_IRQn;
	}
	if (port == GPIOB)
	{
		return (uint32_t)GPIOB_INT_IRQn;
	}
	return SYS_EXTI_INVALID_IRQN;
}

uint8_t SYS_EXTI_LineInGroup(uint32_t pin, uint8_t startLine, uint8_t endLine)
{
	uint8_t lineIndex;

	lineIndex = SYS_EXTI_GetLineIndex(pin);
	if ((lineIndex > 15U) || (lineIndex < startLine) || (lineIndex > endLine))
	{
		return 0U;
	}

	return 1U;
}
