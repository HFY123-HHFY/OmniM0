#include "exti.h"
#include <stdlib.h>
#include "sys.h"
#include "G3507_exti.h"
#include "G3507_Encoder.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/m0p/dl_interrupt.h"


#define API_EXTI_MAX_ID  (16U)

static const API_EXTI_Config_t *s_extiTable;
static uint8_t s_extiCount;
// 多路回调链表头
static API_EXTI_IrqHandlerNode_t *s_extiIrqHandlerList[API_EXTI_MAX_ID] = {0};

static const API_EXTI_Config_t *API_EXTI_FindConfigById(API_EXTI_Id_t id)
{
	uint8_t i;

	if ((s_extiTable == 0) || (s_extiCount == 0U))
	{
		return 0;
	}

	for (i = 0U; i < s_extiCount; ++i)
	{
		if (s_extiTable[i].id == id)
		{
			return &s_extiTable[i];
		}
	}

	return 0;
}

static void API_EXTI_CoreInit(void *port, uint32_t pin, API_EXTI_Trigger_t trigger,
	uint32_t irqn, uint8_t preemptPriority, uint8_t subPriority)
{
	G3507_EXTI_Init(port, pin, trigger, irqn, preemptPriority, subPriority);
}

static uint8_t API_EXTI_CoreIsPendingAndClear(void *port, uint32_t pin)
{
	return G3507_EXTI_IsPendingAndClear(port, pin);
}

static void API_EXTI_DispatchConfig(const API_EXTI_Config_t *config)
{
	if (config == 0)
		return;
	if (API_EXTI_CoreIsPendingAndClear(config->port, config->pin) == 0U)
		return;
	API_EXTI_IrqHandlerNode_t *node = s_extiIrqHandlerList[config->id];
	while (node) {
		if (node->handler)
			node->handler(config->id, node->userData);
		node = node->next;
	}
}

void API_EXTI_Register(const API_EXTI_Config_t *configTable, uint8_t count)
{
	s_extiTable = configTable;
	s_extiCount = count;
}


// 多路注册
int API_EXTI_AddIrqHandler(API_EXTI_Id_t id, API_EXTI_IrqHandler_t handler, void *userData)
{
	if (id >= API_EXTI_MAX_ID || handler == 0)
		return -1;
	API_EXTI_IrqHandlerNode_t *node = (API_EXTI_IrqHandlerNode_t*)malloc(sizeof(API_EXTI_IrqHandlerNode_t));
	if (!node) return -2;
	node->handler = handler;
	node->userData = userData;
	node->next = s_extiIrqHandlerList[id];
	s_extiIrqHandlerList[id] = node;
	return 0;
}

// 注销指定回调
int API_EXTI_RemoveIrqHandler(API_EXTI_Id_t id, API_EXTI_IrqHandler_t handler, void *userData)
{
	if (id >= API_EXTI_MAX_ID || handler == 0)
		return -1;
	API_EXTI_IrqHandlerNode_t **pp = &s_extiIrqHandlerList[id];
	while (*pp) {
		if ((*pp)->handler == handler && (*pp)->userData == userData) {
			API_EXTI_IrqHandlerNode_t *toDel = *pp;
			*pp = toDel->next;
			free(toDel);
			return 0;
		}
		pp = &((*pp)->next);
	}
	return -2;
}

// 清空所有回调
void API_EXTI_ClearIrqHandlers(API_EXTI_Id_t id)
{
	if (id >= API_EXTI_MAX_ID) return;
	API_EXTI_IrqHandlerNode_t *node = s_extiIrqHandlerList[id];
	while (node) {
		API_EXTI_IrqHandlerNode_t *next = node->next;
		free(node);
		node = next;
	}
	s_extiIrqHandlerList[id] = 0;
}

void API_EXTI_Init(API_EXTI_Id_t id, API_EXTI_Trigger_t trigger,
	uint8_t preemptPriority, uint8_t subPriority)
{
	const API_EXTI_Config_t *config;
	uint32_t irqn;

	config = API_EXTI_FindConfigById(id);
	if ((config == 0) || (config->port == 0) || (config->pin == 0U))
	{
		return;
	}

	irqn = SYS_EXTI_GetIrqn(config->port, config->pin);
	if (irqn == SYS_EXTI_INVALID_IRQN)
	{
		return;
	}

	API_EXTI_CoreInit(config->port, config->pin, trigger, irqn, preemptPriority, subPriority);
}

void API_EXTI_HandleIrqByLine(uint8_t lineIndex)
{
	uint8_t i;

	if ((s_extiTable == 0) || (s_extiCount == 0U))
	{
		return;
	}

	for (i = 0U; i < s_extiCount; ++i)
	{
		if (SYS_EXTI_GetLineIndex(s_extiTable[i].pin) == lineIndex)
		{
			API_EXTI_DispatchConfig(&s_extiTable[i]);
		}
	}
}

void API_EXTI_HandleIrqByLineGroup(uint8_t startLine, uint8_t endLine)
{
	uint8_t i;

	if ((s_extiTable == 0) || (s_extiCount == 0U))
	{
		return;
	}

	for (i = 0U; i < s_extiCount; ++i)
	{
		if (SYS_EXTI_LineInGroup(s_extiTable[i].pin, startLine, endLine) != 0U)
		{
			API_EXTI_DispatchConfig(&s_extiTable[i]);
		}
	}
}

void API_EXTI_HandleIrqByPort(void *port)
{
	uint8_t i;

	if ((s_extiTable == 0) || (s_extiCount == 0U) || (port == 0))
	{
		return;
	}

	for (i = 0U; i < s_extiCount; ++i)
	{
		if (s_extiTable[i].port == port)
		{
			API_EXTI_DispatchConfig(&s_extiTable[i]);
		}
	}
}

void GROUP1_IRQHandler(void)
{
	uint32_t pendingGroup;

	/* 先处理编码器外部中断（独立于 API_EXTI 体系） */
	G3507_Encoder_ProcessPortIrq(GPIOA);
	G3507_Encoder_ProcessPortIrq(GPIOB);

	for (;;)
	{
		pendingGroup = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);
		if (pendingGroup == DL_INTERRUPT_GROUP1_IIDX_GPIOA)
		{
			API_EXTI_HandleIrqByPort(GPIOA);
			continue;
		}

		if (pendingGroup == DL_INTERRUPT_GROUP1_IIDX_GPIOB)
		{
			API_EXTI_HandleIrqByPort(GPIOB);
			continue;
		}

		break;
	}
}
