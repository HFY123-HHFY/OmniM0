#include "My_Usart.h"
#include <sys/stat.h>
#include "ti/driverlib/dl_uart_main.h"

/* 系统毫秒计数器（TIMG0 ISR 每 1ms +1，用于帧超时检测） */
extern volatile uint32_t g_sys_tick_ms;

/*
 * 发送环形队列结构：
 * - head: 生产者写入位置（主循环或任务上下文）
 * - tail: 消费者取出位置（TXE 中断上下文）
 */
typedef struct
{
	USART_TypeDef *instance;
	volatile uint16_t head;
	volatile uint16_t tail;
	uint8_t buf[USART_TX_BUF_SIZE];
} USART_TxAsyncQueue;

/* 每个 USART 实例对应一套异步发送队列。 */
static USART_TxAsyncQueue g_usart_tx_q1 = {USART1, 0U, 0U, {0}};
static USART_TxAsyncQueue g_usart_tx_q2 = {USART2, 0U, 0U, {0}};
static USART_TxAsyncQueue g_usart_tx_q3 = {USART3, 0U, 0U, {0}};
static USART_TxAsyncQueue g_usart_tx_q4 = {USART4, 0U, 0U, {0}};

/* 根据 USART 实例返回对应发送队列。 */
static USART_TxAsyncQueue *usart_get_tx_queue(USART_TypeDef *USARTx)
{
	if (USARTx == USART1)
	{
		return &g_usart_tx_q1;
	}
	if (USARTx == USART2)
	{
		return &g_usart_tx_q2;
	}
	if (USARTx == USART3)
	{
		return &g_usart_tx_q3;
	}
	if (USARTx == USART4)
	{
		return &g_usart_tx_q4;
	}
	return 0;
}

/* 统一映射：API 串口 ID -> USART 寄存器实例。 */
static USART_TypeDef *usart_id_to_instance(API_USART_Id_t id)
{
	if (id == API_USART1)
	{
		return USART1;
	}
	if (id == API_USART2)
	{
		return USART2;
	}
	if (id == API_USART3)
	{
		return USART3;
	}
	if (id == API_USART4)
	{
		return USART4;
	}
	return 0;
}

static void usart_disable_tx_irq(USART_TypeDef *USARTx)
{
	DL_UART_Main_disableInterrupt((UART_Regs *)USARTx, DL_UART_MAIN_INTERRUPT_TX);
}

static void usart_enable_tx_irq(USART_TypeDef *USARTx)
{
	DL_UART_Main_enableInterrupt((UART_Regs *)USARTx, DL_UART_MAIN_INTERRUPT_TX);
}

static uint8_t usart_is_tx_irq_enabled(USART_TypeDef *USARTx)
{
	if (DL_UART_Main_getEnabledInterruptStatus((UART_Regs *)USARTx, DL_UART_MAIN_INTERRUPT_TX) != 0U)
	{
		return 1U;
	}
	return 0U;
}

static uint8_t usart_is_tx_ready(USART_TypeDef *USARTx)
{
	if (DL_UART_Main_isTXFIFOFull((UART_Regs *)USARTx) == 0U)
	{
		return 1U;
	}
	return 0U;
}

static uint8_t usart_is_rx_ready(USART_TypeDef *USARTx)
{
	if (DL_UART_Main_isRXFIFOEmpty((UART_Regs *)USARTx) == 0U)
	{
		return 1U;
	}
	return 0U;
}

static uint32_t usart_read_data(USART_TypeDef *USARTx)
{
	return DL_UART_Main_receiveData((UART_Regs *)USARTx);
}

static void usart_write_data(USART_TypeDef *USARTx, uint8_t data)
{
	DL_UART_Main_transmitData((UART_Regs *)USARTx, (uint32_t)data);
}

/* 全局接收解析状态。 */
USART_DataType USART_DataTypeStruct;

/* ── 摄像头数据全局缓存（串口中断 解析后供 PID 环使用）── */
int16_t g_cam_data[CAM_DATA_LEN] = {0};
uint8_t g_cam_count = 0U;

/*
 * parse_buffer_to_int — 把 buffer 中 ASCII 数字字符串转为 int16_t。
 * 支持负数（首个字符为 '-'）。
 * 返回：转换成功返回 true，非法字符返回 false。
 */
static uint8_t parse_buffer_to_int(const uint8_t *buf, uint8_t len, int16_t *out)
{
	uint8_t i;
	uint8_t is_negative;
	int16_t value;

	if ((buf == 0) || (out == 0) || (len == 0U))
	{
		return 0U;
	}

	is_negative = 0U;
	i = 0U;
	value = 0;

	if (buf[0] == '-')
	{
		is_negative = 1U;
		i = 1U;
	}

	for (; i < len; i++)
	{
		if ((buf[i] >= '0') && (buf[i] <= '9'))
		{
			value = (int16_t)(value * 10 + (int16_t)(buf[i] - '0'));
		}
		else
		{
			return 0U; /* 非法字符 */
		}
	}

	if (is_negative != 0U)
	{
		value = (int16_t)(-value);
	}

	*out = value;
	return 1U;
}

/*
 * 把 USARTx 寄存器实例转换为 API 层 ID。
 * 这样可以复用 API_USART_WriteByte 完成阻塞兜底发送。
 */
static uint8_t usart_instance_to_id(USART_TypeDef *USARTx, API_USART_Id_t *id)
{
	if (id == 0)
	{
		return 0U;
	}

	if (USARTx == USART1)
	{
		*id = API_USART1;
		return 1U;
	}
	if (USARTx == USART2)
	{
		*id = API_USART2;
		return 1U;
	}
	if (USARTx == USART3)
	{
		*id = API_USART3;
		return 1U;
	}
	if (USARTx == USART4)
	{
		*id = API_USART4;
		return 1U;
	}
	return 0U;
}

/*
 * 发送 1 字节：
 * 1) 先尝试异步入队；
 * 2) 入队失败（队列满/实例不支持）时，退化为阻塞发送兜底。
 */
void usart_send_byte(USART_TypeDef *USARTx, uint8_t Byte)
{
	API_USART_Id_t id;

	if (usart_send_byte_async(USARTx, Byte) != 0U)
	{
		return;
	}

	if (usart_instance_to_id(USARTx, &id) == 0U)
	{
		return;
	}

	API_USART_WriteByte(id, Byte);
}

/* 异步发送 1 字节：入队后由 TX 中断持续搬运到硬件 FIFO。 */
uint8_t usart_send_byte_async(USART_TypeDef *USARTx, uint8_t Byte)
{
	USART_TxAsyncQueue *q;
	uint16_t nextHead;
	uint8_t txIrqEnabled;

	q = usart_get_tx_queue(USARTx);
	if (q == 0)
	{
		return 0U;
	}

	nextHead = (uint16_t)((q->head + 1U) % USART_TX_BUF_SIZE);
	if (nextHead == q->tail)
	{
		return 0U; /* 队列满，交给上层阻塞兜底发送 */
	}

	q->buf[q->head] = Byte;
	q->head = nextHead;

	txIrqEnabled = usart_is_tx_irq_enabled(USARTx);
	if (txIrqEnabled == 0U)
	{
		/*
		 * 队列从空转非空时使能 TX 中断。
		 * 若当前 FIFO 可写，立即 kick 一次，避免等待下一次硬件事件。
		 */
		usart_enable_tx_irq(USARTx);
		if (usart_is_tx_ready(USARTx) != 0U)
		{
			usart_tx_irq_handler(USARTx);
		}
	}

	return 1U;
}

/* 发送 C 字符串（逐字节调用 usart_send_byte）。 */
void usart_SendString(USART_TypeDef *USARTx, const char *String)
{
	uint16_t i;

	if (String == 0)
	{
		return;
	}

	for (i = 0U; String[i] != '\0'; i++)
	{
		usart_send_byte(USARTx, (uint8_t)String[i]);
	}
}

/* 数字转十进制字符串后发送。 */
void usart_send_number(USART_TypeDef *USARTx, uint32_t Number)
{
	char String[11];

	(void)snprintf(String, sizeof(String), "%lu", (unsigned long)Number);
	usart_SendString(USARTx, String);
}

/* 简单幂函数，供上层保留兼容调用。 */
uint32_t usart_pow(uint32_t X, uint32_t Y)
{
	uint32_t Result;

	Result = 1U;
	while (Y--)
	{
		Result *= X;
	}

	return Result;
}

/* 连续发送字节数组。 */
void usart_send_array(USART_TypeDef *USARTx, uint8_t *Array, uint16_t Length)
{
	uint16_t i;

	if (Array == 0)
	{
		return;
	}

	for (i = 0U; i < Length; i++)
	{
		usart_send_byte(USARTx, Array[i]);
	}
}

/* printf 字符输出重定向。 */
int fputc(int ch, FILE *f)
{
	(void)f;
	usart_send_byte(PRINTF_USART, (uint8_t)ch);
	return ch;
}

/*
 * newlib-nano 的 printf 通常走 _write，而不是逐字符调用 fputc。
 * 实现 _write 后，printf("...") 才会真正从串口输出。
 */
int _write(int file, char *ptr, int len)
{
	int i;

	(void)file;
	if ((ptr == 0) || (len <= 0))
	{
		return 0;
	}

	for (i = 0; i < len; i++)
	{
		usart_send_byte(PRINTF_USART, (uint8_t)ptr[i]);
	}

	return len;
}

/* newlib-nano 最小 syscalls 桩，避免链接阶段未实现告警。 */
int _close(int file)
{
	(void)file;
	return -1;
}

int _fstat(int file, struct stat *st)
{
	(void)file;
	if (st != 0)
	{
		st->st_mode = S_IFCHR;
	}
	return 0;
}

int _getpid(void)
{
	return 1;
}

int _isatty(int file)
{
	(void)file;
	return 1;
}

int _kill(int pid, int sig)
{
	(void)pid;
	(void)sig;
	return -1;
}

int _lseek(int file, int ptr, int dir)
{
	(void)file;
	(void)ptr;
	(void)dir;
	return 0;
}

int _read(int file, char *ptr, int len)
{
	(void)file;
	(void)ptr;
	(void)len;
	return 0;
}

/* 格式化输出：先格式化到本地缓冲，再统一发送。 */
void usart_printf(USART_TypeDef *USARTx, const char *format, ...)
{
	char String[128];
	int len;
	va_list arg;

	va_start(arg, format);
	len = vsnprintf(String, sizeof(String), format, arg);
	va_end(arg);

	if (len <= 0)
	{
		return;
	}

	usart_SendString(USARTx, String);
}

/* TX 中断服务分发函数：当前不走异步发送链路，保留兼容入口。 */
void usart_tx_irq_handler(USART_TypeDef *USARTx)
{
	USART_TxAsyncQueue *q;

	q = usart_get_tx_queue(USARTx);
	if (q == 0)
	{
		return;
	}

	if ((q->tail != q->head) && (usart_is_tx_ready(q->instance) != 0U))
	{
		usart_write_data(q->instance, q->buf[q->tail]);
		q->tail = (uint16_t)((q->tail + 1U) % USART_TX_BUF_SIZE);
	}

	if (q->tail == q->head)
	{
		usart_disable_tx_irq(q->instance);
	}
}

void usart_irq_dispatch_by_id(API_USART_Id_t id, uint32_t *rxData, uint8_t *rxValid)
{
	USART_TypeDef *instance;

	instance = usart_id_to_instance(id);
	if (instance == 0)
	{
		return;
	}

	if ((rxData != 0) && (rxValid != 0))
	{
		*rxValid = 0U;
		if (usart_is_rx_ready(instance) != 0U)
		{
			*rxData = usart_read_data(instance);
			*rxValid = 1U;
		}
	}

	if ((usart_is_tx_irq_enabled(instance) != 0U) && (usart_is_tx_ready(instance) != 0U))
	{
		usart_tx_irq_handler(instance);
	}
}

/*
 * 串口数据包解析（重构版）：
 *
 * 协议格式：s<val1>,<val2>,...,<valN>e
 * 示例：   s88,-93,104e  →  data[0]=88, data[1]=-93, data[2]=104
 *
 * 改进点：
 * - 数字解析提取到 parse_buffer_to_int，消除 ~20 行重复代码
 * - 去掉 memset，仅 reset buffer_len（解析只读到 buffer_len）
 * - data[] 类型统一为 int16_t，不再强转
 * - 记录 start_tick 供帧超时检测使用
 */
void usart_Dispose_Data(USART_TypeDef *USARTx, USART_DataType *p, uint8_t RxData)
{
	(void)USARTx;

	switch (p->state)
	{
	case 0: /* ── 空闲，等待包头 's' ── */
		if (RxData == 's')
		{
			p->state = 1U;
			p->current_index = 0U;
			p->buffer_len   = 0U;
			p->count        = 0U;
			p->start_tick   = g_sys_tick_ms; /* 记录起始时刻 */
		}
		break;

	case 1: /* ── 接收中 ── */
		if (RxData == 'e')
		{
			/* 包尾：解析缓冲区中最后一个数值 */
			if (p->buffer_len > 0U)
			{
				int16_t value;
				if ((parse_buffer_to_int(p->buffer, p->buffer_len, &value) != 0U)
				    && (p->current_index < Data_len))
				{
					p->data[p->current_index] = value;
					p->count = (uint8_t)(p->current_index + 1U);
				}
			}
			p->state = 2U; /* 完成 */
		}
		else if (RxData == ',')
		{
			/* 分隔符：存储当前 buffer 中的数值 */
			if (p->buffer_len > 0U)
			{
				int16_t value;
				if ((parse_buffer_to_int(p->buffer, p->buffer_len, &value) != 0U)
				    && (p->current_index < Data_len))
				{
					p->data[p->current_index] = value;
					p->current_index++;
				}
			}
			p->buffer_len = 0U; /* 准备接收下一个数值 */
		}
		else if (((RxData >= '0') && (RxData <= '9')) || (RxData == '-'))
		{
			/* 数字字符或负号 */
			if (p->buffer_len < 15U)
			{
				/* 负号只能出现在 buffer 开头，否则是非法帧 */
				if ((RxData == '-') && (p->buffer_len != 0U))
				{
					p->state = 0U;
				}
				else
				{
					p->buffer[p->buffer_len++] = RxData;
				}
			}
			else
			{
				p->state = 0U; /* buffer 溢出，丢弃 */
			}
		}
		else
		{
			/* 非法字符，丢弃整帧 */
			p->state = 0U;
		}
		break;

	case 2: /* ── 完成态，等待下一个包头 ── */
		if (RxData == 's')
		{
			p->state = 1U;
			p->current_index = 0U;
			p->count        = 0U;
			p->buffer_len   = 0U;
			p->start_tick   = g_sys_tick_ms;
		}
		break;

	default:
		p->state = 0U;
		break;
	}
}

/*
 * usart_FrameTimeout_Check — 帧超时保护
 *
 * 在 TIMG0 ISR 5ms 槽中周期性调用。
 * 如果解析器在 state=1（正在接收）超过 timeout_ms，
 * 自动复位到 state=0，防止噪声 's' 触发后永久卡死。
 */
void usart_FrameTimeout_Check(USART_DataType *p, uint32_t timeout_ms)
{
	if ((p == 0) || (p->state != 1U))
	{
		return;
	}

	if ((g_sys_tick_ms - p->start_tick) >= timeout_ms)
	{
		p->state = 0U;
	}
}

/* 安全读取解析结果。 */
int16_t USART_Deal(USART_DataType *pData, int8_t index)
{
	if ((pData == 0) || (index < 0) || ((uint8_t)index >= pData->count))
	{
		return 0;
	}

	return pData->data[(uint8_t)index];
}

/*
 * USART_CamCapture — 摄像头数据包捕获
 *
 * 在 USART ISR 回调中检测到 state=2 后调用。
 * 协议格式：s<有效标志>,<X坐标>e（如 s1,240e）
 *
 * g_cam_data[0] (CAM_VALID): 数据有效性标志（1=有效，0=无效）
 * g_cam_data[1] (CAM_X):     摄像头 X 轴坐标
 */
void USART_CamCapture(void)
{
	uint8_t i;
	uint8_t n;

	n = USART_DataTypeStruct.count;
	if (n > CAM_DATA_LEN)
	{
		n = CAM_DATA_LEN;
	}

	for (i = 0U; i < n; i++)
	{
		g_cam_data[i] = USART_DataTypeStruct.data[i];
	}
	g_cam_count = n;

	USART_DataTypeStruct.state = 0U; /* 消费完毕，准备下一帧 */
}
