#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "KEY.h"
#include "Encoder.h"
#include "gray_adc.h"
#include "jy61p.h"

volatile uint8_t key_flag = 0U; /* 按键刷新标志位 */
volatile uint8_t OLED_flag = 0U; /* OLED刷新标志位 */
volatile uint8_t print_task_flag = 0; /* 串口打印任务标志位 */
uint32_t Timer_Bsp_t = 0; /* 时间计数 */

int16_t USART_Packet_Data[USART_PACKET_DATA_LEN] = {0}; /* USART 数据缓存 */
uint8_t USART_Packet_Count = 0; /* USART 数据计数 */

/*
 * TIM1 (1ms): 5ms → 灰度刷新 + 方向环 PID
 */
void Control_Task_TIM_Callback(API_TIM_Id_t id)
{
	static uint8_t dir_5ms_tick = 0U;
	static uint8_t  Encoder_tick = 0U;

	if (id != API_TIM1) 
	{ 
		return; 
	}

	dir_5ms_tick++;
	Encoder_tick++;

	if (dir_5ms_tick >= 5U)
	{
		dir_5ms_tick = 0U;
		GrayADC_Task(&g_graySensor);
		if (!Control_IsTurning())
		{
			Direction_Control();
		}
	}

	if (Encoder_tick >= 20)
	{
		Encoder_tick = 0U;
		G3507_Encoder_SnapshotAll();
		Encoder1_Speed = API_Encoder_GetSpeed(API_ENCODER_1);
		Encoder2_Speed = API_Encoder_GetSpeed(API_ENCODER_2);
		// Direction_Test_Control();
		// PID_Speed_Control((float)Encoder1_Speed, (float)Encoder2_Speed);
		Control_Run((float)Encoder1_Speed, (float)Encoder2_Speed);
	}
}

/*
 * TIM2 (1ms): 20ms → 编码器 + Control_Run
 */
void Control_Task_Encoder_Callback(API_TIM_Id_t id)
{
	static uint8_t  key_tick     = 0U;
	static uint8_t  oled_tick    = 0U;
	static uint8_t  printf_tick  = 0U;
	static uint16_t time_t       = 0U;

	if (id != API_TIM2)
	{
		return;
	}

	/*
	* 按键扫描 */
	Key_Tick();

	key_tick++;
	oled_tick++;
	printf_tick++;
	time_t++;

	if (key_tick >= 10U)
	{
		key_tick = 0U;
		key_flag = 1U;
	}

	if (printf_tick >= 50U)
	{
		printf_tick = 0U;
		print_task_flag = 1U;
	}

	if (oled_tick >= 100U)
	{
		oled_tick = 0U;
		OLED_flag = 1U;
	}

	if (time_t >= 1000U)
	{
		time_t = 0U;
		Timer_Bsp_t++;
	}
}

void Control_Task_USART_Callback(API_USART_Id_t id)
{
	uint32_t data;
	uint8_t rxValid;

	/*
	 * 循环排空 RX FIFO：MSPM0 的 UART 有 4 字节 FIFO，中断按阈值触发，
	 * 不是逐字节触发。必须一次中断把所有可用字节读干净，否则数据堆积→卡顿。
	 */
	do
	{
		data    = 0U;
		rxValid = 0U;
		usart_irq_dispatch_by_id(id, &data, &rxValid);
		if (rxValid != 0U)
		{
			if (id == API_USART3)
			{
				JY61P_RxPush((uint8_t)data);  /* 只入队，解析交给主循环 JY61P_Task() */
				// usart_printf(USART3, "data: %lu\r\n", data);
			}
			// if (id == API_USART1)
			// {
			// 	usart_Dispose_Data(USART1, &USART_DataTypeStruct, (uint8_t)data);
			// }
		}
	} while (rxValid != 0U);
}
