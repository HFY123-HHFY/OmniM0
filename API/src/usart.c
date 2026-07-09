#include "usart.h"
#include "gpio.h"

#include "G3507_hw_config.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_uart_main.h"

static const API_USART_Config_t *s_usartTable;
static uint8_t s_usartCount;
static API_USART_IrqHandler_t s_usartIrqHandlers[API_USART4 + 1U];

static uint8_t API_USART_GetG3507Pinmux(
	API_USART_Id_t id, uint8_t isTx, uint32_t *iomux, uint32_t *func)
{
	if ((iomux == 0) || (func == 0))
	{
		return 0U;
	}

	switch (id)
	{
	case API_USART1:
		if (isTx != 0U)
		{
			*iomux = G3507_USART0_TX_IOMUX;
			*func = G3507_USART0_TX_FUNC;
		}
		else
		{
			*iomux = G3507_USART0_RX_IOMUX;
			*func = G3507_USART0_RX_FUNC;
		}
		return 1U;
	case API_USART2:
		if (isTx != 0U)
		{
			*iomux = G3507_USART1_TX_IOMUX;
			*func = G3507_USART1_TX_FUNC;
		}
		else
		{
			*iomux = G3507_USART1_RX_IOMUX;
			*func = G3507_USART1_RX_FUNC;
		}
		return 1U;
	case API_USART3:
		if (isTx != 0U)
		{
			*iomux = G3507_USART2_TX_IOMUX;
			*func = G3507_USART2_TX_FUNC;
		}
		else
		{
			*iomux = G3507_USART2_RX_IOMUX;
			*func = G3507_USART2_RX_FUNC;
		}
		return 1U;
	case API_USART4:
		if (isTx != 0U)
		{
			*iomux = G3507_USART3_TX_IOMUX;
			*func = G3507_USART3_TX_FUNC;
		}
		else
		{
			*iomux = G3507_USART3_RX_IOMUX;
			*func = G3507_USART3_RX_FUNC;
		}
		return 1U;
	default:
		return 0U;
	}
}

static void API_USART_ConfigTxPin(API_USART_Id_t id)
{
	uint32_t iomux;
	uint32_t func;

	if (API_USART_GetG3507Pinmux(id, 1U, &iomux, &func) == 0U)
	{
		return;
	}
	DL_GPIO_initPeripheralOutputFunction(iomux, func);
}

static void API_USART_ConfigRxPin(API_USART_Id_t id)
{
	uint32_t iomux;
	uint32_t func;

	if (API_USART_GetG3507Pinmux(id, 0U, &iomux, &func) == 0U)
	{
		return;
	}
	DL_GPIO_initPeripheralInputFunction(iomux, func);
}

static void API_USART_CoreInit(uint8_t coreId, uint32_t baudRate)
{
	G3507_USART_Init(coreId, baudRate);
}

static void API_USART_CoreWriteByte(uint8_t coreId, uint8_t data)
{
	G3507_USART_WriteByte(coreId, data);
}

static const API_USART_Config_t *API_USART_FindConfig(API_USART_Id_t id)
{
	uint8_t index;

	if ((s_usartTable == 0) || (s_usartCount == 0U))
	{
		return 0;
	}

	for (index = 0U; index < s_usartCount; ++index)
	{
		if (s_usartTable[index].id == id)
		{
			return &s_usartTable[index];
		}
	}

	return 0;
}

static const API_USART_Config_t *API_USART_FindConfigByCoreId(uint8_t coreId)
{
	uint8_t index;

	if ((s_usartTable == 0) || (s_usartCount == 0U))
	{
		return 0;
	}

	for (index = 0U; index < s_usartCount; ++index)
	{
		if (s_usartTable[index].coreId == coreId)
		{
			return &s_usartTable[index];
		}
	}

	return 0;
}

void API_USART_Register(const API_USART_Config_t *configTable, uint8_t count)
{
	s_usartTable = configTable;
	s_usartCount = count;
}

void API_USART_RegisterIrqHandler(API_USART_Id_t id, API_USART_IrqHandler_t handler)
{
	if ((id < API_USART1) || (id > API_USART4))
	{
		return;
	}

	s_usartIrqHandlers[id] = handler;
}

void API_USART_Init(API_USART_Id_t id, uint32_t baudRate)
{
	const API_USART_Config_t *config;

	config = API_USART_FindConfig(id);
	if (config == 0)
	{
		return;
	}

	if ((config->txPort != 0) && (config->txPin != 0U))
	{
		API_USART_ConfigTxPin(id);
	}

	if ((config->rxPort != 0) && (config->rxPin != 0U))
	{
		API_USART_ConfigRxPin(id);
	}

	if (baudRate == 0U)
	{
		return;
	}

	API_USART_CoreInit(config->coreId, baudRate);
}

void API_USART_WriteByte(API_USART_Id_t id, uint8_t data)
{
	const API_USART_Config_t *config;

	config = API_USART_FindConfig(id);
	if (config == 0)
	{
		return;
	}

	API_USART_CoreWriteByte(config->coreId, data);
}

void API_USART_HandleIrqByCoreId(uint8_t coreId)
{
	const API_USART_Config_t *config;
	API_USART_IrqHandler_t handler;

	config = API_USART_FindConfigByCoreId(coreId);
	if (config == 0)
	{
		return;
	}

	if ((config->id < API_USART1) || (config->id > API_USART4))
	{
		return;
	}

	handler = s_usartIrqHandlers[config->id];
	if (handler == 0)
	{
		return;
	}

	handler(config->id);
}

void UART0_IRQHandler(void)
{
	API_USART_HandleIrqByCoreId(API_USART_CORE_UART0);
}

void UART1_IRQHandler(void)
{
	API_USART_HandleIrqByCoreId(API_USART_CORE_UART1);
}

void UART2_IRQHandler(void)
{
	API_USART_HandleIrqByCoreId(API_USART_CORE_UART2);
}

void UART3_IRQHandler(void)
{
	API_USART_HandleIrqByCoreId(API_USART_CORE_UART3);
}
