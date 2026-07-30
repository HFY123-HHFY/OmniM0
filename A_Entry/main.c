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

	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback);
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task_TIM_Callback);

	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 2：外设硬件初始化
	 * ══════════════════════════════════════════════════════════════════ */
	API_USART_Init(API_USART1, 115200U); /* U1 板载串口调试 */
	API_USART_Init(API_USART2, 115200U); /* U2 步进电机 */
	API_USART_Init(API_USART3, 115200U); /* U3 无线串口调试 */
	API_USART_Init(API_USART4, 115200U); /* U4 摄像头 */
	API_PWM_Init(API_PWM_TIM1, 4000U - 1U, 1U - 1U); /* 电机A相 */
	API_PWM_Init(API_PWM_TIM2, 4000U - 1U, 1U - 1U); /* 电机B相 */
	API_ADC_Init(API_ADC1);
	GrayADC_Init();

	API_I2C_Init();
	API_SPI_Init();
	// App_I2C_ScanOnce();
	// App_SPI_TestOnce();

	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 3：BSP 设备初始化
	 * ══════════════════════════════════════════════════════════════════ */
	LED_Init(LED_LOW);
	KEY_Init();
	OLED_Init(OLED_IF_SPI);
	ICM42688_Init();
	API_Motor_Init();
	API_Encoder_Init(API_ENCODER_1);
	API_Encoder_Init(API_ENCODER_2);
	PID_Control_Init();
	IRLine_Init(&g_irLine);
	StepMotor_Init(0x01);

	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 4：启动系统时基
	 * ══════════════════════════════════════════════════════════════════ */
	API_TIM_Init(API_TIM1, 1U);

	/* ══════════════════════════════════════════════════════════════════
	 * 阶段 5：步进电机使能 + 回零
	 *
	 * 零点校准（仅一次）：
	 *   取消 SetZero 注释 → 烧录 → 手动拨摆杆到零点 → 看到 "Zero OK"
	 *   → 重新注释 SetZero → 再次烧录
	 * ══════════════════════════════════════════════════════════════════ */
	// StepMotor_SetZero(1);
	// usart_printf(USART1, "Zero Set OK\r\n");

	StepMotor_Enable();
	StepMotor_GoHome(3000U);
	StepMotor_ConfigMove(250.0f, 200.0f);  // 最大 250RPM，加速 200RPM/s, 设置步进电机转速

	// PID_EncoderSpeed_Set(&speed_loop, 50.0f, 100.0f, 0.0f, 20.0f); /* 速度环 */
	// Set_PID(&direction_pid,  0.50f, 0.15f, 0.010f);/* 循迹环 */
	// YawPid_Set(2.0f, 0.3f, 0.0f, 45.0f); /* 偏航环 */

	Buzzer_Beep(100);

	while (1)
	{
		/* ── 蜂鸣器/LED 调度 @5ms ── */
		if (tasks.buzzer_5ms.flag)
		{
			tasks.buzzer_5ms.flag = false;
			Buzzer_Task();
		}

		/* ── 按键同步 @20ms（KEY2 选任务在此更新 s_task_select）── */
		if (tasks.key_20ms.flag)
		{
			tasks.key_20ms.flag = false;
			key_Get();

			/* 步进电机手动测试（调试用，注释掉避免和 Task_Run 冲突） */
			// if (Key == 1) { Key = 0; StepMotor_SetAngle(180.0f);  }
			// if (Key == 2) { Key = 0; StepMotor_SetAngle(0.0f);    }
			// if (Key == 3) { Key = 0; StepMotor_SetAngle(-180.0f); }
		}

		/* ── 串口打印 50ms ── */
#if (DEBUG_PRINT_ENABLE == 1U)
		if (tasks.print_50ms.flag)
		{
			tasks.print_50ms.flag = false;
			// GrayADC_PrintBits(&g_graySensor, USART1); 
			// GrayADC_PrintRaw(&g_graySensor, USART3); /* 校准感为灰度 */ 
		}
#endif

		/* ── OLED 刷新 100ms ── */
#if (DEBUG_OLED_ENABLE == 1U)
		if (tasks.oled_100ms.flag)
		{
			tasks.oled_100ms.flag = false;
			OLED_Clear();
			OLED_Printf(0,  0, OLED_6X8, "T%d", s_task_select); /* 当前任务 */
			OLED_Printf(32, 0, OLED_6X8, "%lus", Task_2_GetLapTime()); /* Task_2 圈时 */
			// OLED_Printf(0, 16, OLED_6X8, "%d", g_cam_data[0]); /* 摄像头 X 坐标 */
			OLED_Printf(0, 16, OLED_6X8, "yaw: %.2f", g_icm42688.yaw); /* 偏航角 */
			OLED_Printf(0, 48, OLED_6X8, "L%+d R%+d", Encoder1_Speed, Encoder2_Speed); /* 左右电机速度 */

			OLED_Printf(64, 0, OLED_6X8, "%d%d%d%d%d%d%d%d",
			g_graySensor.digital_bits[0], g_graySensor.digital_bits[1],
			g_graySensor.digital_bits[2], g_graySensor.digital_bits[3],
			g_graySensor.digital_bits[4], g_graySensor.digital_bits[5],
			g_graySensor.digital_bits[6], g_graySensor.digital_bits[7]); /* 灰度传感器数字量 */

			// OLED_Printf(64, 0, OLED_6X8, "%d%d%d%d%d%d%d%d",
			// g_graySensor.digital_bits[0], g_graySensor.digital_bits[1],
			// g_graySensor.digital_bits[2], g_graySensor.digital_bits[3],
			// g_graySensor.digital_bits[4], g_graySensor.digital_bits[5],
			// g_graySensor.digital_bits[6], g_graySensor.digital_bits[7]); /* 红外传感器数字量 */

			// OLED_Printf(32, 0, OLED_6X8, "Y%+d", yaw_pid.output);
			OLED_Printf(0, 48, OLED_6X8, "D%+d", direction_pid.output);
			// OLED_Printf(0, 16, OLED_6X8, "R%+.1f P%+.1f", g_icm42688.roll, g_icm42688.pitch);
			// OLED_Printf(0, 32, OLED_6X8, "Y%+.3f", g_icm42688.yaw);
			OLED_Update();
		}
#endif
	}
}
