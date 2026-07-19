/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*系统sys层*/
#include "sys.h"
#include "Delay.h"

/*API层 MCU片内外设*/
#include "usart.h"
#include "tim.h"
#include "pwm.h"
#include "adc.h"
#include "Encoder.h"

/*app应用层*/
#include "My_Usart/My_Usart.h"
#include "API_I2C.h"
#include "API_SPI.h"
#include "PID/PID.h"
#include "Control/Control.h"
#include "Control_Task/Control_Task.h"
#include "Tasks/Tasks.h"     /* Task_GetSelect / Task_GetActive */

/*BSP硬件抽象层*/
#include "LED.h"
#include "Buzzer.h"   /* Buzzer_Task / Buzzer_Beep */
#include "KEY.h"
#include "OLED.h"
#include "Control.h"
#include "TB6612.h"
#include "gray_adc.h"
#include "MPU6050.h"
#include "MPU6050_Int.h"
#include "jy61p.h"

/* ── 调试开关 ── */
#define DEBUG_PRINT_ENABLE  0U   /* 开启/关闭串口 printf 调试输出 */
#define DEBUG_OLED_ENABLE   1U   /* 开启/关闭 OLED 显示            */

int main(void)
{
/* 系统时钟配置初始化 */
	SYS_Init();
/* 注册层：注册相关资源，登记资源映射 */
	Enroll_USART_Register();				/* USART 资源注册 */
	Enroll_PWM_Register();					/* PWM 资源注册 */
	Enroll_ADC_Register();					/* ADC 资源注册 */
	Enroll_TIM_Register();					/* TIM 资源注册 */
	Enroll_I2C_Register();					/* I2C 资源注册 */
	Enroll_SPI_Register();					/* SPI 资源注册 */
	Enroll_LED_Register();					/* LED 资源注册 */
	Enroll_KEY_Register();					/* KEY 资源注册 */
	Enroll_OLED_Register();					/* OLED SPI 控制脚注册 */
	Enroll_TB6612_Register();				/* TB6612 资源注册 */
	Enroll_Encoder_Register();				/* 编码器 资源注册 */
	Enroll_GrayADC_Register();				/* GrayADC 灰度传感器 资源注册 */

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); 	/* USART 中断回调注册 */
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task_TIM_Callback);   /* TIMG0 1ms 时基回调 */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1，板载串口调试
	API_USART_Init(API_USART2, 115200U); // 初始化 USART2，无线串口调试
	API_USART_Init(API_USART3, 115200U); // 初始化 USART3
	API_USART_Init(API_USART4, 115200U); // 初始化 USART4，JY61P 陀螺仪
	API_PWM_Init(API_PWM_TIM1, 2000U - 1U, 1U - 1U); /* TIMG8@ULPCLK 40MHz: 40M/1/2000 = 20kHz，占空比 0-2000 */
	API_ADC_Init(API_ADC1); // 初始化 ADC1
	GrayADC_Init();							/* GrayADC 硬件 + digital_bits 全白（必须在 TIM 前） */
	API_TIM_Init(API_TIM1, 1U); /* TIMG0 系统时基：每 1ms 触发一次中断 */

/* 通信协议初始化 */
	API_I2C_Init();						/* 软件 I2C 初始化 */
	API_SPI_Init();						/* 软件 SPI 初始化 */
	// App_I2C_ScanOnce();				/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

/* BSP硬件抽象层初始化*/
	LED_Init(LED_LOW); // 初始化LED-低电平
	KEY_Init(); // 初始化按键
	OLED_Init(OLED_IF_SPI);		 			/* OLED_IF_I2C(4针) / OLED_IF_SPI(7针) */
	JY61P_Init();							/* JY61P 陀螺仪数据结构初始化 */
	TB6612_Init(); 							/* TB6612 电机驱动初始化 */
	API_Encoder_Init(API_ENCODER_1); 		/* 编码器 1 初始化 */
	API_Encoder_Init(API_ENCODER_2); 		/* 编码器 2 初始化 */
	PID_Control_Init();						/* PID 结构初始化（dt/死区/积分分离） */
	JY61P_ZAxisZero(); /* 当前朝向设为 0°，阻塞约 3.5 秒 */
	Buzzer_Beep(200);  /* 蜂鸣器短鸣 200ms，非阻塞 */

	while (1)
	{
		/* ── JY61P 数据解析 @5ms ── */
		if (tasks.jy61p_5ms.flag)
		{
			tasks.jy61p_5ms.flag = false;
			JY61P_Task();
		}

		/* ── 蜂鸣器/LED 调度 @5ms ── */
		if (tasks.buzzer_5ms.flag)
		{
			tasks.buzzer_5ms.flag = false;
			Buzzer_Task();
		}

		/* ── 按键轮询 @20ms（消抖在 ISR 1ms Key_Tick 完成）── */
		if (tasks.key_20ms.flag)
		{
			tasks.key_20ms.flag = false;
			key_Get();
		}

		/* 串口打印 50ms */
#if (DEBUG_PRINT_ENABLE == 1U)
		if (tasks.print_50ms.flag)
		{
			tasks.print_50ms.flag = false;
			usart_printf(USART2, "Angle:%.3f",JY61P_GetYawFiltered());
			GrayADC_PrintBits(&g_graySensor, USART2);
		}
#endif

		/* OLED 刷新 100ms */
#if (DEBUG_OLED_ENABLE == 1U)
		if (tasks.oled_100ms.flag)
		{
			tasks.oled_100ms.flag = false;
			OLED_Clear();
			OLED_Printf(0, 0, OLED_6X8, "T:%d Y%4.1f",Task_GetSelect(),JY61P_GetYawFiltered());
			OLED_Printf(78, 0, OLED_6X8, "%d%d%d%d%d%d%d%d",
				g_graySensor.digital_bits[0], g_graySensor.digital_bits[1],
				g_graySensor.digital_bits[2], g_graySensor.digital_bits[3],
				g_graySensor.digital_bits[4], g_graySensor.digital_bits[5],
				g_graySensor.digital_bits[6], g_graySensor.digital_bits[7]);
			OLED_Printf(0, 16, OLED_6X8, "L:%d  R:%d", Encoder1_Speed, Encoder2_Speed);
			OLED_Printf(64, 16, OLED_6X8, "P:%d T:%d", Task_GetPos(), Task_GetActive());
			OLED_Printf(0, 32, OLED_6X8, "H:%d", s_gray_enter_fired);
			OLED_Printf(24, 32, OLED_6X8, "B:%d", s_gray_exit_fired);
			OLED_Printf(64, 32, OLED_6X8, "OUT:%d", (int)yaw_pid.output);
			OLED_Update();
		}
#endif
	}
}
