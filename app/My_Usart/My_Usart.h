#ifndef __MY_USART_H
#define __MY_USART_H

/*
 * My_Usart 模块说明：
 * 1) 统一提供“应用层可直接调用”的串口发送/printf/数据包解析接口；
 * 2) 保持与原标准库封装接近的函数名和调用方式；
 * 3) 底层适配当前工程 API 层(usart.h)与 G3507 寄存器视图。
 */

#include "Enroll.h"
#include "usart.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 异步发送环形缓冲区大小（字节）。 */
#ifndef USART_TX_BUF_SIZE
#define USART_TX_BUF_SIZE 512U
#endif

/* printf 默认输出串口（可在编译参数或上层头文件中重定义）。 */
#ifndef PRINTF_USART
#define PRINTF_USART USART1
#endif

/* CR1.TXEIE：发送寄存器空中断使能位。 */
#ifndef USART_CR1_TXEIE
#define USART_CR1_TXEIE (1UL << 7)
#endif

/* 解析数据包后最多保存的数据项个数。 */
#define Data_len 10U

typedef G3507_USART_View_t USART_TypeDef;

/*
 * 串口数据包解析状态结构：
 * 协议格式：s<val1>,<val2>,...,<valN>e
 *
 * 摄像头协议示例：s1,240e
 *   - 数据1 (CAM_VALID): 有效标志（1=有效，0=无效）
 *   - 数据2 (CAM_X):     摄像头 X 轴坐标（自然单位）
 *
 * - 's'：包头
 * - ','：分隔符
 * - 'e'：包尾
 *
 * data[] 改为 int16_t，解析结果直接存储有符号值，
 * 避免 uint16_t ↔ int16_t 来回强转导致负值被扭曲。
 */
typedef struct
{
	int16_t data[Data_len];
	uint8_t count;
	uint8_t state;
	uint8_t current_index;
	uint8_t buffer[16];
	uint8_t buffer_len;
	uint32_t start_tick;       /* 收到 's' 时的系统 tick，用于帧超时检测 */
} USART_DataType;

/* 全局解析状态实例，建议在中断中喂数据，在主循环中读取结果。 */
extern USART_DataType USART_DataTypeStruct;

/* ── 摄像头数据全局缓存（USART4 解析后使用）── */
#define CAM_DATA_LEN  4U
extern int16_t g_cam_data[CAM_DATA_LEN];
extern uint8_t g_cam_count;

/* 摄像头数据包字段别名（协议格式：s<有效标志>,<X坐标>e） */
#define CAM_VALID    g_cam_data[0]   /* 数据有效性标志（1=有效，0=无效） */
#define CAM_X        g_cam_data[1]   /* 摄像头识别到的 X 轴坐标         */

/*
 * 发送单字节（优先异步，不行则退化阻塞发送）。
 * 参数：USARTx 选择串口实例，Byte 为待发送字节。
 */
void usart_send_byte(USART_TypeDef *USARTx, uint8_t Byte);

/*
 * 发送单字节（纯异步）。
 * 返回：1=入队成功，0=队列满或串口不支持。
 */
uint8_t usart_send_byte_async(USART_TypeDef *USARTx, uint8_t Byte);

/* 发送以 '\0' 结尾的字符串。 */
void usart_SendString(USART_TypeDef *USARTx, const char *String);

/* 发送 32 位无符号整数（十进制字符串形式）。 */
void usart_send_number(USART_TypeDef *USARTx, uint32_t Number);

/* 幂函数：返回 X^Y。 */
uint32_t usart_pow(uint32_t X, uint32_t Y);

/* 连续发送数组中的 Length 个字节。 */
void usart_send_array(USART_TypeDef *USARTx, uint8_t *Array, uint16_t Length);

/* printf 重定向接口：默认发送到 PRINTF_USART。 */
int fputc(int ch, FILE *f);

/*
 * 类 printf 串口输出。
 * 用法：usart_printf(USART1, "rpm=%d\r\n", rpm);
 */
void usart_printf(USART_TypeDef *USARTx, const char *format, ...);

/*
 * TXE 中断处理函数：
 * 需要在对应 USARTx_IRQHandler 的 TXE 分支里调用。
 */
void usart_tx_irq_handler(USART_TypeDef *USARTx);

/* 根据 API 串口 ID 处理 RX/TX 中断事件（由注册层回调触发）。 */
void usart_irq_dispatch_by_id(API_USART_Id_t id, uint32_t *rxData, uint8_t *rxValid);

/*
 * 接收数据包解析函数：
 * 建议在 RXNE 分支读取到 data 后调用。
 */
void usart_Dispose_Data(USART_TypeDef *USARTx, USART_DataType *USART_DataTypeStruct, uint8_t RxData);

/*
 * 获取已解析数据项。
 * 返回：索引有效则返回数据，否则返回 0。
 */
int16_t USART_Deal(USART_DataType *pData, int8_t index);

/*
 * 帧超时检测：若解析器在 state=1 超过 timeout_ms 仍未收到 'e'，
 * 自动复位到 state=0（防止噪声触发 's' 后永久卡死）。
 * 建议在 TIMG0 ISR 5ms 槽中调用。
 */
void usart_FrameTimeout_Check(USART_DataType *pData, uint32_t timeout_ms);

/*
 * 摄像头数据包捕获：把 USART_DataTypeStruct 中解析好的数据
 * 复制到 g_cam_data[] 全局数组，并消费状态机（state 归零）。
 * 在 USART 中断回调中检测到 state=2 后调用。
 */
void USART_CamCapture(void);

#endif
