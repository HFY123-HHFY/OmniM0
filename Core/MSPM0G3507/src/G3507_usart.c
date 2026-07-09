#include "G3507_usart.h"
#include "IrqPriority.h"

#include "G3507_sys.h"

#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_uart_main.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"

#define G3507_USART_CORE_UART3 (3U)

typedef struct
{
	UART_Regs *regs;
	IRQn_Type irq;
} G3507_USART_Map_t;

static G3507_USART_Map_t G3507_USART_GetMap(uint8_t usartId)
{
	G3507_USART_Map_t map;

	map.regs = 0;
	map.irq = NonMaskableInt_IRQn;

	switch (usartId)
	{
	case 0U:
		map.regs = UART0;
		map.irq = UART0_INT_IRQn;
		break;
	case 1U:
		map.regs = UART1;
		map.irq = UART1_INT_IRQn;
		break;
	case 2U:
		map.regs = UART2;
		map.irq = UART2_INT_IRQn;
		break;
	case 3U:
		map.regs = UART3;
		map.irq = UART3_INT_IRQn;
		break;
	default:
		break;
	}

	return map;
}

/*
 * UART 波特率分频输入时钟选择：
 * - UART0/1/2 位于 PD0，沿用 BusClk(当前工程下约为 MCLK/2)；
 * - UART3 位于 PD1，其功能时钟与 PD0 不同，按 MCLK 计算波特率。
 */
static uint32_t G3507_USART_GetFunctionalClockHz(uint8_t usartId)
{
	if (usartId == G3507_USART_CORE_UART3)
	{
		return G3507_SYS_GetMclkHz();
	}

	return G3507_SYS_GetBusClkHz();
}

void G3507_USART_Init(uint8_t usartId, uint32_t baudRate)
{
	G3507_USART_Map_t map;
	DL_UART_Main_ClockConfig clockConfig;
	DL_UART_Main_Config uartConfig;
	uint32_t uartClockHz;

	map = G3507_USART_GetMap(usartId);
	if ((map.regs == 0) || (baudRate == 0UL))
	{
		return;
	}

	if (!DL_UART_Main_isPowerEnabled(map.regs))
	{
		DL_UART_Main_reset(map.regs);
		DL_UART_Main_enablePower(map.regs);
		while (!DL_UART_Main_isPowerEnabled(map.regs))
		{
		}
	}

	DL_UART_Main_disable(map.regs);

	clockConfig.clockSel = DL_UART_MAIN_CLOCK_BUSCLK;
	clockConfig.divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1;
	DL_UART_Main_setClockConfig(map.regs, &clockConfig);

	uartConfig.mode = DL_UART_MAIN_MODE_NORMAL;
	uartConfig.direction = DL_UART_MAIN_DIRECTION_TX_RX;
	uartConfig.flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE;
	uartConfig.parity = DL_UART_MAIN_PARITY_NONE;
	uartConfig.wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS;
	uartConfig.stopBits = DL_UART_MAIN_STOP_BITS_ONE;
	DL_UART_Main_init(map.regs, &uartConfig);

	uartClockHz = G3507_USART_GetFunctionalClockHz(usartId);
	if (uartClockHz == 0UL)
	{
		uartClockHz = 32000000UL;
	}
	DL_UART_Main_configBaudRate(map.regs, uartClockHz, baudRate);

	DL_UART_Main_enableInterrupt(map.regs, DL_UART_MAIN_INTERRUPT_RX);
		/*
		 * RX FIFO 阈值设为 1 字节 — 每收到 1 个字节立即触发中断。
		 * 默认值（1/2 满 = 2 字节）会导致帧尾字节卡在 FIFO 里，
		 * 严重拖慢 JY61P 等连续数据流传感器的响应速度。
		 */
		DL_UART_Main_setRXFIFOThreshold(map.regs,
		                                DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_Main_enable(map.regs);
/* 根据 USART 实例选择优先级：USART3(2U)→PRIO 2, 其余→PRIO 3 */
		{
			uint32_t prio = IRQ_PRIO_DEFAULT;
			if (usartId == 2U)      { prio = IRQ_PRIO_USART3; } /* JY61P */
			else if (usartId == 0U) { prio = IRQ_PRIO_USART1; } /* 调试 */
			else if (usartId == 1U) { prio = IRQ_PRIO_USART2; }
			else if (usartId == 3U) { prio = IRQ_PRIO_USART4; }
			NVIC_SetPriority(map.irq, prio);
		}
	NVIC_EnableIRQ(map.irq);
}

void G3507_USART_WriteByte(uint8_t usartId, uint8_t data)
{
	G3507_USART_Map_t map;

	map = G3507_USART_GetMap(usartId);
	if (map.regs == 0)
	{
		return;
	}

	DL_UART_Main_transmitDataBlocking(map.regs, (uint8_t)data);
}
