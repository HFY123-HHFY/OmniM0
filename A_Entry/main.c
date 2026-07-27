/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*系统sys层*/
#include "sys.h"
#include "Delay.h"
#include "BusRate.h"       /* 软件总线统一配置中心 */

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
#include "Tasks/Tasks.h"

/*BSP硬件抽象层*/
#include "LED.h"
#include "Buzzer.h"
#include "KEY.h"
#include "OLED.h"
#include "Control.h"
#include "TB6612.h"
#include "API_Motor.h"
#include "ICM42688.h"
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
	// Enroll_TB6612_Register();				/* TB6612 资源注册 */
	Enroll_Encoder_Register();				/* 编码器 资源注册 */
	Enroll_GrayADC_Register();				/* GrayADC 灰度传感器 资源注册 */

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); 	/* USART 中断回调注册 */
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task_TIM_Callback);   /* TIMG0 1ms 时基回调 */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1-板载串口调试
	API_USART_Init(API_USART2, 115200U); // 初始化 USART2-JY61P 陀螺仪
	API_USART_Init(API_USART3, 115200U); // 初始化 USART3-无线串口调试
	API_USART_Init(API_USART4, 115200U); // 初始化 USART4-预留串口
	API_PWM_Init(API_PWM_TIM1, 4000U - 1U, 1U - 1U); /* TIMG7@PD1 电机A相 */
	API_PWM_Init(API_PWM_TIM2, 4000U - 1U, 1U - 1U); /* TIMG6@PD1 电机A相 */
	API_PWM_Init(API_PWM_TIM3, 4000U - 1U, 1U - 1U); /* TIMA0@PD1 电机B相 */
	// API_PWM_Init(API_PWM_TIM4, 4000U - 1U, 1U - 1U); /* TIMA1@PD1 预留PWM */
	// API_PWM_Init(API_PWM_TIM5, 2000U - 1U, 1U - 1U); /* TIMG8@PD0 预留PWM(非必要不启动) */
	API_ADC_Init(API_ADC1); 				// 初始化 ADC1
	GrayADC_Init();							/* GrayADC 硬件 + digital_bits 全白（必须在 TIM 前） */

/* 通信协议初始化 */
	API_I2C_Init();						/* 软件 I2C 初始化 */
	API_SPI_Init();						/* 软件 SPI 初始化 */
	// App_I2C_ScanOnce();				/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

/* BSP硬件抽象层初始化*/
	LED_Init(LED_LOW); // 初始化LED-低电平
	KEY_Init(); // 初始化按键
	OLED_Init(OLED_IF_SPI);		 			/* OLED_IF_I2C(4针) / OLED_IF_SPI(7针) */
	uint8_t icmOk = ICM42688_Init();	/* ICM42688 陀螺仪（SPI2, 5MHz） */
	usart_printf(USART1, "ICM42688=%d\r\n", icmOk);
	JY61P_Init();							/* JY61P 陀螺仪数据结构初始化 */
	// TB6612_Init(); 						/* TB6612 电机驱动初始化（已换用 AT4950） */
	API_Motor_Init();						/* 电机驱动上电刹车（当前=AT4950） */
	API_Encoder_Init(API_ENCODER_1); 		/* 编码器 1 初始化 */
	API_Encoder_Init(API_ENCODER_2); 		/* 编码器 2 初始化 */
	PID_Control_Init();						/* PID 结构初始化（dt/死区/积分分离） */

	// JY61P_ZAxisZero(); /* 当前朝向设为 0°，阻塞约 3.5 秒（CPU 循环延时，不依赖 tick） */

	API_TIM_Init(API_TIM1, 1U); /* TIMG0 1ms ISR 启动 —— 所有硬件已就绪 */
	Buzzer_Beep(200);           /* 蜂鸣器短鸣 200ms，非阻塞（依赖 g_sys_tick_ms） */

	// PID_EncoderSpeed_Set(&speed_loop, 20.0f, 150.0f, 0.0f, 15.0f);
	// Set_PID(&direction_pid,  1.0f, 0.002f, 0.01f);      /* 灰度方向环参数    */
	// YawPid_Set(0.2f, 0.00f, 0.0f, -45.0f);  /* 旋转专用 PID + A 点目标 */

	float icmRoll = 0.0f, icmPitch = 0.0f, icmYaw = 0.0f;
	// float icmGz = 0.0f;

	while (1)
	{
		ICM42688_GetAttitude(&icmRoll, &icmPitch, &icmYaw); /* 读姿态角 */
		// ICM42688_GetGyroscope(0, 0, &icmGz);  /* 只读Z轴角速度 */

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

		if (Key == 1U)
		{
			LED_Control(LED1, LED_HIGH);
			API_Motor_SetSpeed(3000, 3000);
		}
		if (Key == 2U)
		{
			LED_Control(LED2, LED_HIGH);
			API_Motor_SetSpeed(0, 3000);
		}
		if (Key == 3U)
		{
			LED_Control(LED3, LED_HIGH);
			API_Motor_SetSpeed(-3000, -3000);
		}
		if (Key == 4U)
		{
			LED_Control(LED1, LED_LOW);
			LED_Control(LED2, LED_LOW);
			LED_Control(LED3, LED_LOW);
			API_Motor_SetSpeed(0,0);
		}
		/* 串口打印 50ms */
#if (DEBUG_PRINT_ENABLE == 1U)
		if (tasks.print_50ms.flag)
		{
			tasks.print_50ms.flag = false;
			// usart_printf(USART1, "R:%.2f P:%.2f Y:%.2f\r\n", icmRoll, icmPitch, icmYaw);
			GrayADC_PrintRaw(&g_graySensor, USART3);
			// usart_printf(USART1, "key: %lu\r\n", Key);
		}
#endif

		/* OLED 刷新 100ms */
#if (DEBUG_OLED_ENABLE == 1U)
		if (tasks.oled_100ms.flag)
		{
			tasks.oled_100ms.flag = false;
			
			OLED_Printf(0, 0, OLED_6X8, "T:%d JY%.1f", Task_GetSelect(), JY61P_GetYawFiltered() * 0.01f);
			OLED_Printf(0, 16, OLED_6X8, "R:%.3f   P:%.3f", icmRoll, icmPitch);
			OLED_Printf(0, 32, OLED_6X8, "Y:%.3f", icmYaw);

			// OLED_Printf(0, 32, OLED_6X8, "Y:%+5.1f Gz:%+5.0f", icmYaw, icmGz);
			OLED_Printf(78, 48, OLED_6X8, "%d%d%d%d%d%d%d%d",
				g_graySensor.digital_bits[0], g_graySensor.digital_bits[1],
				g_graySensor.digital_bits[2], g_graySensor.digital_bits[3],
				g_graySensor.digital_bits[4], g_graySensor.digital_bits[5],
				g_graySensor.digital_bits[6], g_graySensor.digital_bits[7]);
			OLED_Update();
		}
#endif
	}
}
