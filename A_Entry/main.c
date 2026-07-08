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

/*BSP硬件抽象层*/
#include "LED.h"
#include "KEY.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MPU6050_Int.h"
#include "Control.h"
#include "TB6612.h"
#include "gray_adc.h"

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
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); /* USART 中断回调注册 */
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task_TIM_Callback);             /* TIM1: PID */
	API_TIM_RegisterIrqHandler(API_TIM2, Control_Task_Encoder_Callback);         /* TIM2: Encoder 杂项任务 */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1，波特率 115200
	API_USART_Init(API_USART2, 115200U); // 初始化 USART2，波特率 115200
	API_USART_Init(API_USART3, 115200U); // 初始化 USART3，波特率 115200
	API_PWM_Init(API_PWM_TIM1, 400U - 1U, 8U - 1U); /* 10kHz */
	API_ADC_Init(API_ADC1); // 初始化 ADC1
	API_TIM_Init(API_TIM1, 1U); /* TIM1: PID 节拍， 每 1ms */
	API_TIM_Init(API_TIM2, 1U); /* TIM2: 编码器节拍，每 1ms*/

/* 通信协议初始化 */
	API_I2C_Init();						/* 软件 I2C 初始化 */
	API_SPI_Init();						/* 软件 SPI 初始化 */
	// App_I2C_ScanOnce();					/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

/*BSP硬件抽象层初始化*/
	LED_Init(LED_LOW); // 初始化LED-低电平
	KEY_Init(); // 初始化按键
	OLED_Init(OLED_IF_SPI);		 		/* OLED_IF_I2C(4针) / OLED_IF_SPI(7针) */

	// MPU_Init();
	// uint8_t mpu6050_dma_int = mpu_dmp_init();
	// usart_printf(USART1, "mpu6050_dma_int= %d\r\n", mpu6050_dma_int);
	// Enroll_MPU6050_Register();				/* MPU6050 INT 资源注册（DMP 初始化后才能使能 EXTI） */

	GrayADC_Init();							/* GrayADC 灰度传感器初始化（地址引脚） */
	TB6612_Init(); 							/* TB6612 电机驱动初始化 */
	API_Encoder_Init(API_ENCODER_1); 		/* 编码器 1 初始化 */
	API_Encoder_Init(API_ENCODER_2); 		/* 编码器 2 初始化 */
	PID_Control_Init();						/* PID 结构初始化（dt/死区/积分分离） */

	/* 速度环：左右轮各自 PID，20ms 周期，输出限幅 = TB6612_MAX_DUTY */
	PID_EncoderSpeed_Set(&speed_loop, 1.4f, 35.0f, 0.0f, 20.0f);
	/*                       		   kp    ki    kd  目标速度    */
	/* 方向环：线位置 PID，5ms 周期，Out_max=180 留给速度环余量 */
	// Set_PID(&direction_pid, 0.02f, 0.01f, 0.0f);
	Set_PID(&direction_pid, 0.035f, 0.0001f, 0.005f);
	/*                        kp      ki      kd                   */
	/*                               Integral_max  Out_max         */
	LED_Turn(Buzzer1, 200U);				/* 蜂鸣器短鸣 */

/* ── 调试开关：开启/关闭所有 printf ── */
#define DEBUG_PRINT_ENABLE  1U
/* ── 调试开关：开启/关闭所有 OLED显示 ── */
#define DEBUG_OLED_ENABLE   1U

	while (1)
	{
		/* MPU6050 DMP */
		if (mpu_flag == 1U)
		{
			mpu_flag = 0U;
			// mpu_dmp_get_data(&Pitch, &Roll, &Yaw); 
			// MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
			/* MPU_Get_Accelerometer(&aacx, &aacy, &aacz); */
		}
		/* KEY 控制*/
		key_Get();

/* 串口数据打印 */
		#if (DEBUG_PRINT_ENABLE == 1U)
			if (print_task_flag != 0U)
			{
				print_task_flag = 0U;
				// usart_printf(USART1, "key: %lu\r\n", Key);
				// usart_printf(USART1, "Pitch=%.2f Roll=%.2f Yaw=%.2f\r\n", Pitch, Roll, Yaw);
				GrayADC_PrintLinePos(&g_graySensor, USART3);
			}
		#endif

/* OLED数据打印 */
		#if (DEBUG_OLED_ENABLE == 1U) 
			OLED_Clear();
			OLED_Printf(0, 0, OLED_8X16, "%d", Timer_Bsp_t);
			OLED_Printf(64, 0, OLED_8X16, "%d", Key);
			// OLED_Printf(0, 16, OLED_8X16, "%.1f  %.1f  %.1f", Pitch, Roll, Yaw);
			OLED_Printf(0, 16, OLED_8X16, "Dir PID: %.1f", direction_pid.output);
			OLED_Printf(0, 32, OLED_8X16, "L %d  R %d", Encoder1_Speed, Encoder2_Speed);
			OLED_Update();
		#endif
	}
}
