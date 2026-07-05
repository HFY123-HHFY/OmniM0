#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "KEY.h"
#include "Encoder.h"
#include "gray_adc.h"

/* printf节拍 */
volatile uint8_t print_task_flag = 0;
/* 程序运行的时间戳（s） */
uint32_t Timer_Bsp_t = 0;

/* 串口数据包解析结果缓存（新包到达时自动刷新） */
int16_t USART_Packet_Data[USART_PACKET_DATA_LEN] = {0};
uint8_t USART_Packet_Count = 0;

/*
 * 定时器回调函数：
 * 由 API_TIM 的通用中断分发层在更新中断到来后调用。
 *
 * 调度频率设计：
 *   TIM1 (1ms):
 *     ├─ 2ms tick — 预留
 *     └─ 5ms tick — GrayADC_Task 传感器刷新 + 方向环 PID → g_steer
 *
 *   TIM2 (1ms):
 *     ├─ 20ms tick — 编码器快照 + 速度环 + 读 g_steer 融合 → TB6612
 *     ├─ 50ms tick — 串口打印 flag
 *     └─ 1s   tick — 时间戳
 */
void Control_Task_TIM_Callback(API_TIM_Id_t id)
{
	static uint8_t pid_2ms_tick  = 0U;
	static uint8_t dir_5ms_tick  = 0U;

	if (id != API_TIM1)
	{
		return;
	}

	pid_2ms_tick++;
	dir_5ms_tick++;

	if (pid_2ms_tick >= 2U)
	{
		pid_2ms_tick = 0U;
	}

	if (dir_5ms_tick >= 5U)
	{
		dir_5ms_tick = 0U;

		/* 5ms: 刷新灰度传感器 + 方向环 PID → 更新全局 g_steer */
		GrayADC_Task(&g_graySensor);
		Direction_Control();
	}
}

/*
 * API_TIM2: 1ms -> Encoder 20ms
 */
void Control_Task_Encoder_Callback(API_TIM_Id_t id)
{
	static uint8_t Encoder_tick = 0U;
	static uint8_t printf_tick = 0U;
	static uint16_t time_t = 0U;

	if (id != API_TIM2)
	{
		return;
	}
	// 按键扫描
	Key_Tick();

	Encoder_tick++;
	printf_tick++;
	time_t++;

	if (Encoder_tick >= 20)
	{
		Encoder_tick = 0U;

		G3507_Encoder_SnapshotAll();
		Encoder1_Speed = API_Encoder_GetSpeed(API_ENCODER_1);
		Encoder2_Speed = API_Encoder_GetSpeed(API_ENCODER_2);

		/* 20ms: 循线控制 → TB6612 */
		/* 正式: LineFollow_Output()   测速度: PID_Speed_Control()   测方向: Direction_Test_Control() */
		Direction_Test_Control();
	}

	if (printf_tick >= 50U)
	{
		printf_tick = 0U;
		print_task_flag = 1U;
	}

	if (time_t >= 1000U)
	{
		time_t = 0U;
		Timer_Bsp_t++;
	}
}

/*
 * USART 中断回调协议解析：
 * 协议格式：s12,-34,56e
 */
void Control_Task_USART_Callback(API_USART_Id_t id)
{
	uint32_t data;
	uint8_t rxValid;
	data = 0U;
	rxValid = 0U;
	usart_irq_dispatch_by_id(id, &data, &rxValid);
	if (rxValid != 0U)
	{
		if (id == API_USART1)
		{
			usart_Dispose_Data(USART1, &USART_DataTypeStruct, (uint8_t)data);
		}
	}
}
