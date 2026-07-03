#ifndef __API_USART_H
#define __API_USART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "G3507_usart.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"

/* G3507 中 API_USART1/2/3 分别映射 UART0/1/2。 */

typedef enum
{
	API_USART1 = 1U,
	API_USART2 = 2U,
	API_USART3 = 3U,
} API_USART_Id_t;

#define API_USART_CORE_UART0   (0U)
#define API_USART_CORE_UART1   (1U)
#define API_USART_CORE_UART2   (2U)
#define API_USART_CORE_UART3   (3U)

typedef struct
{
	API_USART_Id_t id;
	uint8_t coreId;
	void *txPort;
	uint32_t txPin;
	void *rxPort;
	uint32_t rxPin;
} API_USART_Config_t;

typedef void (*API_USART_IrqHandler_t)(API_USART_Id_t id);

/*
 * 供应用层在串口中断里直接访问的最小寄存器视图。
 * 这样 main.c 可以直接写 USARTx_IRQHandler，而不用再走额外的 API 串口 IRQ 包装。
 */
typedef UART_Regs G3507_USART_View_t;

#define USART1 ((G3507_USART_View_t *)UART0)
#define USART2 ((G3507_USART_View_t *)UART1)
#define USART3 ((G3507_USART_View_t *)UART2)
#define USART4 ((G3507_USART_View_t *)UART3)
/* G3507 串口状态位兼容常量。 */
#define USART_SR_RXNE (1UL << 5)
#define USART_SR_TC   (1UL << 6)
#define USART_SR_TXE  (1UL << 7)

void API_USART_Register(const API_USART_Config_t *configTable, uint8_t count);
void API_USART_RegisterIrqHandler(API_USART_Id_t id, API_USART_IrqHandler_t handler);
void API_USART_HandleIrqByCoreId(uint8_t coreId);
/* 串口初始化接口：id 选择串口，baudRate 设置波特率。 */
void API_USART_Init(API_USART_Id_t id, uint32_t baudRate);
/* 串口发送 1 字节。 */
void API_USART_WriteByte(API_USART_Id_t id, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __API_USART_H */
