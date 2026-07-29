#include "Control.h"

#include "My_Usart/My_Usart.h"          /* usart_printf */
#include "Control_Task/Control_Task.h"   /* NonBlockDelay_t */
#include "pwm.h"
#include "API_Motor.h"
#include "KEY.h"
#include "LED.h"
#include "jy61p.h"     /* JY61P_GetData（保留，ICM42688 为主力 IMU） */
#include "ICM42688.h"  /* ICM42688_GetSnapshot — 偏航角主数据源  */
#include "Encoder.h"   /* Encoder1_Speed / Encoder2_Speed    */

/* ── 前向声明 ── */
void Motor(void);   /* 小球轨道倾斜电机（预留，Ball_Move_Control 调用） */

/* =========================================================================
 * PID 对象
 * ========================================================================= */
/* 速度环 */
PID_EncoderSpeed_t speed_loop;

/* 方向环（灰度循线位置 PID） */
PID_TypeDef direction_pid;

/* 偏航角位置环 PID */
PID_TypeDef yaw_pid;

/* 小球位置环 PID（摄像头 X 坐标 → 电机控制） */
PID_TypeDef ball_pid;

/* 灰度传感器实例（Control.h 中 extern，供 Control_Task / main 引用） */
GrayADC_Sensor_t g_graySensor;

/* 亚博红外传感器实例 — 当前切回感为灰度，暂时注释 */

/*
 * PID_Control_Init — 速度环 + 方向环 + 偏航角环结构初始化。
 */
void PID_Control_Init(void)
{
    /* ── 速度环结构（左右轮独立 PID）── */
    PID_EncoderSpeed_Init(&speed_loop);
    PID_SetSampleTime(&speed_loop.left,  20);   /* dt = 20ms */
    PID_SetSampleTime(&speed_loop.right, 20);

    /* ── 方向环结构 ── */
    PID_Init(&direction_pid);
    PID_Init_WithLimit(&direction_pid, 500, API_MOTOR_MAX_DUTY);
    PID_SetSampleTime(&direction_pid, 5);
    PID_SetDeadband(&direction_pid, 60);
    PID_SetIntegralSeparation(&direction_pid, 3000);

    /* ── 偏航角环结构 ── */
    YawPid_Init();

    /* ── 小球位置环结构 ── */
    BallPid_Init();
}

/*
 * 偏航角位置环 — ICM42688 snap.yaw 控制小车朝向（20ms）
*/
void YawPid_Init(void)
{
    PID_Init(&yaw_pid);
    PID_Init_WithLimit(&yaw_pid, 500, API_MOTOR_MAX_DUTY);    /* I_out ±500, Out_max=4000 */
    PID_SetSampleTime(&yaw_pid, 20);             /* dt = 20ms，和速度环同频     */
    PID_SetDeadband(&yaw_pid, 100);              /* ±1.0° 死区（100 cdeg）      */
}

/*
 * YawPid_InitStraight — 直走专用偏航角 PID。
 *
 * 与默认版的区别：
 *   - 输出上限从 4000 降到 1200（不给速度环造成剧烈差速扰动）
 *   - 死区从 1.5° 放宽到 2.0°，避免微小角度波动反复纠偏
 *   - 积分限从 ±500 降到 ±150（防积分超调引发震荡）
 */
void YawPid_InitStraight(void)
{
    PID_Init(&yaw_pid);
    PID_Init_WithLimit(&yaw_pid, 150, 1200);       /* I_out ±150, Out_max ±1200    */
    PID_SetSampleTime(&yaw_pid, 20);              /* dt = 20ms                  */
    PID_SetDeadband(&yaw_pid, 200);               /* ±2.0° 死区（200 cdeg）     */
}

/*
 * YawError_Wrap — 角度误差（含 ±180° 回绕）。
 * 内部函数，用户不直接调用。单位：cdeg（×100）。
 */
static int32_t YawError_Wrap(int32_t target, int32_t current)
{
    int32_t error = target - current;
    while (error >  18000) { error -= 36000; }
    while (error < -18000) { error += 36000; }
    return error;
}

/*
 * YawPid_SetTarget — 设置偏航角目标（Init 阶段，非 ISR）。
 *
 * @param degrees  目标角度（度），例：90.0f, -45.5f
 */
void YawPid_SetTarget(float degrees)
{
    int32_t cdeg = (int32_t)(degrees * 100.0f + (degrees >= 0.0f ? 0.5f : -0.5f));
    PID_SetTarget(&yaw_pid, cdeg);
}

/*
 * YawPid_Set PID 参数 + 目标角度。
 */
void YawPid_Set(float kp, float ki, float kd, float target_deg)
{
    Set_PID(&yaw_pid, kp, ki, kd);
    YawPid_SetTarget(target_deg);
}

/*
 * YawPid_Calc — 偏航角 PID 计算（20ms ISR）。
 *
 * @param yaw_degrees  当前偏航角（度），来自 g_icm42688.yaw
 * @return             steer 值（±4000），正值=右转
 *
 * 内部自动：yaw → cdeg → ±180° wrap → 纯整数 PID_Calc。
 */
int32_t YawPid_Calc(float yaw_degrees)
{
    int32_t yaw_cdeg    = (int32_t)(yaw_degrees * 100.0f);
    int32_t target_cdeg = yaw_pid.Target;       /* Target 存的就是 cdeg，非 Q16.16 */
    int32_t error       = YawError_Wrap(target_cdeg, yaw_cdeg);
    int32_t eff_target  = yaw_cdeg + error;
    yaw_pid.Target      = eff_target;           /* 回存 cdeg（过 ±180° 会自然回绕） */
    return PID_Calc(&yaw_pid, yaw_cdeg);
}

/* 方向环输出暂存（Direction_Control 写，LineFollow_Output 读） */
static int32_t g_steer = 0;

/* =========================================================================
 * Direction_Control — 方向环（TIMG0 ISR 5ms）
 *
 * 灰度线位置 → 方向 PID → g_steer（整数，全部 Q16.16 计算在 PID 库内完成）
 * ========================================================================= */
void Direction_Control(void)
{
    int32_t pos    = GrayADC_LinePosition(&g_graySensor);
    int32_t center = (int32_t)(7U * GRAY_ADC_SENSOR_SPACING_MM * 100U / 2U);

    if (pos < 0)
    {
        return; /* 传感器未校准/丢线，保持上一拍 steer */
    }

    PID_SetTarget(&direction_pid, center);
    g_steer = -PID_Calc(&direction_pid, pos);  /* PID_Calc 返回 int32_t */

    }

/* =========================================================================
 * MotorOutput_Clamp — 电机输出限幅到 API_MOTOR_MAX_DUTY (±4000)
 * ========================================================================= */
void MotorOutput_Clamp(int16_t *left, int16_t *right)
{
    if (*left  >  (int16_t)API_MOTOR_MAX_DUTY) *left  =  (int16_t)API_MOTOR_MAX_DUTY;
    if (*left  < -(int16_t)API_MOTOR_MAX_DUTY) *left  = -(int16_t)API_MOTOR_MAX_DUTY;
    if (*right >  (int16_t)API_MOTOR_MAX_DUTY) *right =  (int16_t)API_MOTOR_MAX_DUTY;
    if (*right < -(int16_t)API_MOTOR_MAX_DUTY) *right = -(int16_t)API_MOTOR_MAX_DUTY;
}

/* =========================================================================
 * LineFollow_Output — 速度环 + 灰度方向环融合输出（TIMG0 ISR 20ms）
 *
 * 内部直读 Encoder1/2_Speed 作为速度反馈。
 * steer>0 → 右转（左轮加速、右轮减速）。
 * ========================================================================= */
void LineFollow_Output(void)
{
    int32_t out_left  = 0;
    int32_t out_right = 0;
    int16_t left, right;

    /* ── 1. 速度环（整数 PID）── */
    PID_EncoderSpeed_Control(&speed_loop,
                             (int32_t)Encoder1_Speed,
                             (int32_t)Encoder2_Speed,
                             &out_left, &out_right);

    /* ── 2. 融合方向环 g_steer ── */
    left  = (int16_t)out_left  - (int16_t)g_steer;
    right = (int16_t)out_right + (int16_t)g_steer;

    /* ── 3. 限幅 + 写电机 ── */
    MotorOutput_Clamp(&left, &right);
    API_Motor_SetSpeed(left, right);
}

/* =========================================================================
 * Drive_YawSpeed — 速度环 + 偏航角环融合输出（TIMG0 ISR 20ms）
 *
 * 偏航角数据源：ICM42688 双缓冲原子快照（保证 roll/yaw 同帧）。
 * yaw_steer>0 → 右转（左轮加速、右轮减速）。
 * ========================================================================= */
void Drive_YawSpeed(void)
{
    ICM42688_Data_t snap;
    ICM42688_GetSnapshot(&snap);         /* 双缓冲原子读：9 字段来自同一 SPI 帧 */
    float   yaw       = snap.yaw;        /* 偏航角 (°)                        */
    int32_t yaw_steer = YawPid_Calc(yaw);
    int32_t out_left  = 0;
    int32_t out_right = 0;
    int16_t left, right;

    PID_EncoderSpeed_Control(&speed_loop,
                             (int32_t)Encoder1_Speed,
                             (int32_t)Encoder2_Speed,
                             &out_left, &out_right);

    left  = (int16_t)out_left  + (int16_t)yaw_steer;
    right = (int16_t)out_right - (int16_t)yaw_steer;

    MotorOutput_Clamp(&left, &right);
    API_Motor_SetSpeed(left, right);
}

/*
 * 速度环独立控制（纯速度模式）。
 */
void PID_Speed_Control(void)
{
    int32_t out_left  = 0;
    int32_t out_right = 0;
    int16_t left, right;

    PID_EncoderSpeed_Control(&speed_loop, (int32_t)Encoder1_Speed, (int32_t)Encoder2_Speed, &out_left, &out_right);

    left  = (int16_t)out_left;
    right = (int16_t)out_right;

    MotorOutput_Clamp(&left, &right);
    API_Motor_SetSpeed(left, right);
}

/*
 * 灰度方向环单独测试 — 纯差速转向。
 */
void Direction_Test_Control(void)
{
    int16_t left  = -(int16_t)g_steer;
    int16_t right = (int16_t)g_steer;

    MotorOutput_Clamp(&left, &right);
    API_Motor_SetSpeed(left, right);
}

/*
 * YawTest_Control — 偏航角单独测试（ICM42688 数据源）
 */
void YawTest_Control(void)
{
    ICM42688_Data_t snap;
    ICM42688_GetSnapshot(&snap);
    float   yaw   = snap.yaw;
    int32_t steer = YawPid_Calc(yaw);
    int16_t left  = (int16_t)steer;
    int16_t right = -(int16_t)steer;
    MotorOutput_Clamp(&left, &right);
    API_Motor_SetSpeed(left, right);
}

/* ══════════════════════════════════════════════════════════════════════
 * 小球位置环 — 摄像头 X 坐标 → 位置 PID → 电机
 *
 * 物理背景：半圆弧轨道，X 轴坐标 -12 ~ +12。
 * 摄像头识别小球 X 位置写入 g_cam_data[0]，PID 输出控制电机倾斜轨道。
 *
 * BallPid_Init   — 结构体初始化（20ms 采样周期，±4000 输出上限）
 * BallPid_Calc   — 给定当前 X 坐标，返回 PID 输出（natural units）
 * Ball_Move_Control — 完整控制周期：读取 g_cam_data[0] → PID → Motor()
 * ══════════════════════════════════════════════════════════════════════ */

void BallPid_Init(void)
{
    PID_Init(&ball_pid);
    PID_Init_WithLimit(&ball_pid, 500, API_MOTOR_MAX_DUTY);  /* I_max=500, Out_max=4000 */
    PID_SetSampleTime(&ball_pid, 20);                         /* 20ms，和 ISR 任务周期一致 */
    PID_SetDeadband(&ball_pid, 1);                            /* ±1 单位死区，摄像头分辨率级 */
}

/* BallPid_SetTarget — 设置小球位置环目标 X 坐标 */
void BallPid_SetTarget(int32_t target_x)
{
    PID_SetTarget(&ball_pid, target_x);
}

/* BallPid_Calc — 计算小球位置环 PID 输出 */
int32_t BallPid_Calc(int16_t ball_x)
{
    return PID_Calc(&ball_pid, (int32_t)ball_x);
}

/*
 * Ball_Move_Control — 小球位置环完整控制周期（TIMG0 ISR 20ms）。
 *
 * 内部：g_cam_data[0] → PID_Calc → ball_pid.output → Motor()。
 * Motor() 预留，当前由具体任务配置电机输出。
 */
void Ball_Move_Control(void)
{
    BallPid_Calc(g_cam_data[0]);   /* PID 计算，结果存入 ball_pid.output */
    Motor();                        /* 电机输出（预留，读取 ball_pid.output） */
}

/* ══════════════════════════════════════════════════════════════════════
 * 电机加载 — 小球轨道倾斜电机
 *
 * 由 Ball_Move_Control() 调用，读取 ball_pid.output 驱动电机。
 * 当前为预留空函数，待电机硬件确定后填充。
 * ══════════════════════════════════════════════════════════════════════ */
void Motor(void)
{
    /* TODO: 根据 ball_pid.output 驱动轨道倾斜电机                     */
    /* int16_t duty = (int16_t)ball_pid.output;                       */
    /* API_Motor_SetSpeed(duty, duty);  — 或其他电机配置              */
}
