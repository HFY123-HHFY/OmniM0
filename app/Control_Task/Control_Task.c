#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "KEY.h"
#include "Encoder.h"
#include "gray_adc.h"

volatile uint8_t print_task_flag = 0;
uint32_t Timer_Bsp_t = 0;
int16_t USART_Packet_Data[USART_PACKET_DATA_LEN] = {0};
uint8_t USART_Packet_Count = 0;

/*
 * TIM1 (1ms): 5ms → 灰度刷新 + 方向环 PID
 */
void Control_Task_TIM_Callback(API_TIM_Id_t id)
{
	static uint8_t dir_5ms_tick = 0U;

	if (id != API_TIM1) 
	{ 
		return; 
	}

	dir_5ms_tick++;
	if (dir_5ms_tick >= 5U)
	{
		dir_5ms_tick = 0U;
		GrayADC_Task(&g_graySensor);
		if (!Control_IsTurning())
		{
			Direction_Control();
		}
	}
}

/*
 * TIM2 (1ms): 20ms → 编码器 + Control_Run
 */
void Control_Task_Encoder_Callback(API_TIM_Id_t id)
{
	static uint8_t  Encoder_tick = 0U;
	static uint8_t  printf_tick  = 0U;
	static uint16_t time_t       = 0U;

	if (id != API_TIM2)
	{
		return;
	}

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
		Control_Run((float)Encoder1_Speed, (float)Encoder2_Speed);
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
