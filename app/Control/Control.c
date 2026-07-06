#include "Control.h"

#include "My_Usart/My_Usart.h"          /* usart_printf */
#include "pwm.h"
#include "TB6612.h"
#include "KEY.h"

/* =========================================================================
 * PID 对象
 * ========================================================================= */
/* 速度环 */
PID_EncoderSpeed_t speed_loop;

/* 方向环（灰度循线位置 PID） */
PID_TypeDef direction_pid;

/* 灰度传感器实例（Control.h 中 extern，供 Control_Task / main 引用） */
GrayADC_Sensor_t g_graySensor;

/*
 * PID_Control_Init — 速度环 + 方向环 结构初始化。
 *
 * 只做 dt / deadband / integral_separation 等调度相关配置。
 * kp / ki / kd / Out_max / target 等参数请到 main.c 里设置，方便调参。
 */
void PID_Control_Init(void)
{
	/* ── 速度环结构 ── */
	PID_EncoderSpeed_Init(&speed_loop);
	PID_SetSampleTime(&speed_loop.left,  0.02f);
	PID_SetSampleTime(&speed_loop.right, 0.02f);

	/* ── 方向环结构 ── */
	PID_Init(&direction_pid);
	PID_Init_WithLimit(&direction_pid, 50000.0f, 180.0f);
	PID_SetSampleTime(&direction_pid, 0.005f);
	PID_SetDeadband(&direction_pid, 60.0f);
	PID_SetIntegralSeparation(&direction_pid, 3000.0f);
}

/* 方向环输出暂存（Direction_Control 写，LineFollow_Output 读） */
static float g_steer = 0.0f;

/* =========================================================================
 * MotorOutput_Clamp — 电机输出限幅到 TB6612_MAX_DUTY (±400)
 * ========================================================================= */
void MotorOutput_Clamp(int16_t *left, int16_t *right)
{
	if (*left  >  (int16_t)TB6612_MAX_DUTY) { *left  =  (int16_t)TB6612_MAX_DUTY; }
	if (*left  < -(int16_t)TB6612_MAX_DUTY) { *left  = -(int16_t)TB6612_MAX_DUTY; }
	if (*right >  (int16_t)TB6612_MAX_DUTY) { *right =  (int16_t)TB6612_MAX_DUTY; }
	if (*right < -(int16_t)TB6612_MAX_DUTY) { *right = -(int16_t)TB6612_MAX_DUTY; }
}

/* =========================================================================
 * Direction_Control — 方向环（TIM1 5ms）
 *
 * 只做传感器读取 + 方向 PID 计算，结果存入 g_steer。
 * 不写 TB6612，硬件输出统一由 LineFollow_Output 负责。
 * ========================================================================= */
void Direction_Control(void)
{
	int32_t pos    = GrayADC_LinePosition(&g_graySensor);
	int32_t center = (int32_t)(7U * GRAY_ADC_SENSOR_SPACING_MM * 100U / 2U);

	if (pos < 0)
	{
		return; /* 传感器未校准/丢线，保持上一拍 steer */
	}

	PID_SetTarget(&direction_pid, (float)center);
	g_steer = -PID_Calc(&direction_pid, (float)pos); /* 负号：线在左(pos<center)→error>0→PID>0→steer<0→左转回线 */
}

/* =========================================================================
 * LineFollow_Output — 速度环 + 方向环融合输出（TIM2 20ms）
 *
 * steer>0 → 左轮加速、右轮减速 → 车右转（纠正"线偏右"）
 * steer<0 → 左轮减速、右轮加速 → 车左转（纠正"线偏左"）
 *
 * 正式循线用。唯一写 TB6612 的入口。
 * ========================================================================= */
void LineFollow_Output(float actual_left, float actual_right)
{
	float   out_left  = 0.0f;
	float   out_right = 0.0f;
	int16_t left      = 0;
	int16_t right     = 0;

	/* ── 1. 速度环 ── */
	PID_EncoderSpeed_Control(&speed_loop, actual_left, actual_right,
	                         &out_left, &out_right);

	/* ── 2. 融合方向环 steer ── */
	left  = (int16_t)out_left  - (int16_t)g_steer;
	right = (int16_t)out_right + (int16_t)g_steer;

	/* ── 3. 限幅 + 死区 → 写电机 ── */
	MotorOutput_Clamp(&left, &right);
	TB6612_SetSpeed(left, right);
}

/*
 * 速度环独立控制（纯速度模式，不使用方向环 steer）。
 */
void PID_Speed_Control(float actual_left, float actual_right)
{
	float   out_left  = 0.0f;
	float   out_right = 0.0f;
	int16_t left, right;

	PID_EncoderSpeed_Control(&speed_loop, actual_left, actual_right,
	                         &out_left, &out_right);

	left  = (int16_t)out_left;
	right = (int16_t)out_right;

	MotorOutput_Clamp(&left, &right);
	TB6612_SetSpeed(left, right);
}

/*
 * 方向环单独测试 — 纯差速转向，绕过速度环。
 */
void Direction_Test_Control(void)
{
	int16_t left  = -(int16_t)g_steer;
	int16_t right = (int16_t)g_steer;

	MotorOutput_Clamp(&left, &right);
	TB6612_SetSpeed(left, right);
}
