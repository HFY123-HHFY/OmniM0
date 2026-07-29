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
#include "API_Motor.h"
#include "ICM42688.h"
#include "gray_adc.h"
#include "IR_Line.h"
#include "StepMotor.h"
#include "MPU6050.h"
#include "MPU6050_Int.h"

/* ── 调试开关 ── */
#define DEBUG_PRINT_ENABLE  1U
#define DEBUG_OLED_ENABLE   1U

int main(void)
{
	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 1：系统时钟 + 资源注册
	 * ══════════════════════════════════════════════════════════════════ */
	SYS_Init();

	Enroll_USART_Register();
	Enroll_PWM_Register();
	Enroll_ADC_Register();
	Enroll_TIM_Register();
	Enroll_I2C_Register();
	Enroll_SPI_Register();
	Enroll_LED_Register();
	Enroll_KEY_Register();
	Enroll_OLED_Register();
	Enroll_TB6612_Register();
	Enroll_Encoder_Register();
	Enroll_GrayADC_Register();

	/* 注册后绑定中断回调（此时硬件尚未初始化，只登记函数指针） */
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback);
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task_TIM_Callback);

	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 2：外设硬件初始化（按依赖顺序：先通信口，再器件）
	 * ══════════════════════════════════════════════════════════════════ */
	API_USART_Init(API_USART1, 115200U);  /* 调试串口            */
	API_USART_Init(API_USART2, 115200U);  /* K230 摄像头通信      */
	API_USART_Init(API_USART3, 115200U);  /* 步进电机 / 无线调试   */
	API_USART_Init(API_USART4, 115200U);  /* 八路红外灰度         */
	API_PWM_Init(API_PWM_TIM1, 4000U - 1U, 1U - 1U);
	API_PWM_Init(API_PWM_TIM2, 4000U - 1U, 1U - 1U);
	API_ADC_Init(API_ADC1);
	GrayADC_Init();                       /* 必须在 TIM 前 */

	API_I2C_Init();
	API_SPI_Init();
	// App_I2C_ScanOnce();				/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 3：BSP 设备初始化（按依赖：GPIO → SPI/I2C → 传感器 → 电机）
	 * ══════════════════════════════════════════════════════════════════ */
	LED_Init(LED_LOW);
	KEY_Init();
	OLED_Init(OLED_IF_SPI);
	ICM42688_Init();                      /* 返回值忽略，无传感器也不阻塞 */
	API_Motor_Init();
	API_Encoder_Init(API_ENCODER_1);
	API_Encoder_Init(API_ENCODER_2);
	PID_Control_Init();
	IRLine_Init(&g_irLine);
	StepMotor_Init(0x01);

	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 4：启动系统时基 → 所有硬件就绪 → 使能中断
	 * ══════════════════════════════════════════════════════════════════ */
	API_TIM_Init(API_TIM1, 1U);           /* TIMG0 1ms ISR 启动 */

	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 5：步进电机使能 + 回零（依赖 TIMG0 的 g_sys_tick_ms 做超时）
	 *
	 * 校准零点（仅一次，用完立即注释！）：
	 *   1. 取消 SetZero 注释 → 烧录 → 手动把摆杆拨到零点位置
	 *   2. 看到 "Zero Set OK" → 重新注释 → 再次烧录
	 * ══════════════════════════════════════════════════════════════════ */
	// StepMotor_SetZero(1);
	// usart_printf(USART1, "Zero Set OK\r\n");

	StepMotor_Enable();                   /* 使能（阻塞 ~500ms 等应答）  */
	StepMotor_GoHome(5000U);              /* 回零（阻塞等到位，最长 5s） */
	StepMotor_ConfigMove(600.0f, 400.0f); /* 最大转速 600RPM，加速度 400RPM/s */

	Buzzer_Beep(100);                     /* 短鸣 100ms = 初始化完成     */

	while (1)
	{
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
			if (Key == 1) { Key = 0; StepMotor_SetAngle(180.0f);  }
			if (Key == 2) { Key = 0; StepMotor_SetAngle(0.0f);    }
			if (Key == 3) { Key = 0; StepMotor_SetAngle(-180.0f); }
		}

		/* ── 串口打印 50ms ── */
#if (DEBUG_PRINT_ENABLE == 1U)
		if (tasks.print_50ms.flag)
		{
			tasks.print_50ms.flag = false;
			// IRLine_PrintBits(&g_irLine, USART1);
		}
#endif

		/* ── OLED 刷新 100ms ── */
#if (DEBUG_OLED_ENABLE == 1U)
		if (tasks.oled_100ms.flag)
		{
			tasks.oled_100ms.flag = false;
			OLED_Clear();

			OLED_Printf(64, 0, OLED_6X8, "%d%d%d%d%d%d%d%d",
			g_graySensor.digital_bits[0], g_graySensor.digital_bits[1],
			g_graySensor.digital_bits[2], g_graySensor.digital_bits[3],
			g_graySensor.digital_bits[4], g_graySensor.digital_bits[5],
			g_graySensor.digital_bits[6], g_graySensor.digital_bits[7]); // 灰度8路

			// OLED_Printf(64, 0, OLED_6X8, "%d%d%d%d%d%d%d%d",
            // g_irLine.digital_bits[0], g_irLine.digital_bits[1],
            // g_irLine.digital_bits[2], g_irLine.digital_bits[3],
            // g_irLine.digital_bits[4], g_irLine.digital_bits[5],
            // g_irLine.digital_bits[6], g_irLine.digital_bits[7]); // 红外8路

			OLED_Printf(0,  0, OLED_6X8, "T%d", s_task_select);
			OLED_Printf(32, 0, OLED_6X8, "%lus", Task_2_GetLapTime());
			OLED_Printf(0, 16, OLED_6X8, "%+d", g_cam_data[0]);
			OLED_Printf(0, 48, OLED_6X8, "L%+d R%+d", Encoder1_Speed, Encoder2_Speed);
			OLED_Update();
		}
#endif
	}
}
