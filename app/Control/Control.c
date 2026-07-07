#include "Control.h"

#include "My_Usart/My_Usart.h"          /* usart_printf */
#include "pwm.h"
#include "TB6612.h"
#include "KEY.h"
#include "LED.h"

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
	if (*left  >  (int16_t)TB6612_MAX_DUTY)
	{
		*left  =  (int16_t)TB6612_MAX_DUTY;
	}
	if (*left  < -(int16_t)TB6612_MAX_DUTY)
	{
		*left  = -(int16_t)TB6612_MAX_DUTY;
	}
	if (*right >  (int16_t)TB6612_MAX_DUTY)
	{
		*right =  (int16_t)TB6612_MAX_DUTY;
	}
	if (*right < -(int16_t)TB6612_MAX_DUTY)
	{
		*right = -(int16_t)TB6612_MAX_DUTY;
	}
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
	g_steer = -PID_Calc(&direction_pid, (float)pos);
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
	left  = (int16_t)out_left  + (int16_t)g_steer;
	right = (int16_t)out_right - (int16_t)g_steer;

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
	int16_t left  = (int16_t)g_steer;
	int16_t right = -(int16_t)g_steer;

	MotorOutput_Clamp(&left, &right);
	TB6612_SetSpeed(left, right);
}

/* ══════════════════════════════════════════════════════════════════════
 * Control_Run — 所有循线控制逻辑的入口
 * ══════════════════════════════════════════════════════════════════════ */

#define TURN_BRAKE_MS   5U   /* 刹车时长：发现路口后双轮急刹多久，杀前进惯量 */
#define TURN_PIVOT_MS   150U   /* 差速转弯时长：逆时针 pivot 多久，决定转的角度 */
#define TURN_DEBOUNCE   1U     /* 路口去抖拍数：连续多少拍(20ms/拍)左2路见黑才触发 */

/* ↓ 以下由上面 3 个自动算出，不用改 ↓ */
#define TURN_BRAKE_TICK (TURN_BRAKE_MS  / 20U)  /* 刹车拍数 (1拍=20ms) */
#define TURN_PIVOT_TICK (TURN_PIVOT_MS  / 20U)  /* 差速转弯拍数 */
#define TURN_TOTAL_TICK (TURN_BRAKE_TICK + TURN_PIVOT_TICK)  /* 总拍数：刹完转到此值退出 */

static uint8_t  s_running   = 0U;  /* 0=停车, 1=运行 */
static uint8_t  s_turning   = 0U;  /* 0=直走, 1=转弯 */
static uint8_t  s_turn_db   = 0U;  /* 去抖计数 */
static uint16_t s_turn_tick = 0U;  /* 转弯时序 (20ms/拍) */

uint8_t Control_IsTurning(void)
{
	return s_turning;
}

void Control_Run(float actual_left, float actual_right)
{
	/* ── 按键 ── */
	if (Key == 2U && s_running == 0U)
	{
		s_running = 1U;
		s_turning = 0U;
		s_turn_db = s_turn_tick = 0U;
		PID_Reset(&direction_pid);
		PID_Reset(&speed_loop.left);
		PID_Reset(&speed_loop.right);
	}
	else if (Key == 3U)
	{
		s_running = 0U;
		s_turning = 0U;
		s_turn_db = s_turn_tick = 0U;
		TB6612_SetSpeed(0, 0);
		PID_Reset(&direction_pid);
		PID_Reset(&speed_loop.left);
		PID_Reset(&speed_loop.right);
	}

	if (s_running == 0U)
	{
		return;
	}

	/* ── 转弯 / 直走 ── */
	if (s_turning)
	{
		if (s_turn_tick < TURN_BRAKE_TICK)
			TB6612_SetSpeed(-200, -200);
		else if (s_turn_tick < TURN_TOTAL_TICK)
			TB6612_SetSpeed(-200,  350);
		else
		{
			s_turning = 0U;
			PID_Reset(&direction_pid);
			PID_Reset(&speed_loop.left);
			PID_Reset(&speed_loop.right);
		}
		s_turn_tick++;
	}
	else
	{
		/* 路口检测 */
		if (g_graySensor.digital_bits[0] == 0U &&
		    g_graySensor.digital_bits[1] == 0U)
		{
			LED_Control(LED2, LED_HIGH);
			s_turn_db++;
			if (s_turn_db >= TURN_DEBOUNCE)
				{ s_turning = 1U; s_turn_tick = 0U; s_turn_db = 0U; }
		}
		else { s_turn_db = 0U; }

		LineFollow_Output(actual_left, actual_right);
	}
}
