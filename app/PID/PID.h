#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════════════════
 * 整数 PID 库（M0+ 无 FPU 优化）
 *
 * ── 架构 ──
 *   配置层（Set_PID / PID_EncoderSpeed_Set）：接受 float，一次性转 Q16.16。
 *     只在启动时调用 2-3 次，浮点开销可忽略（~30µs）。
 *
 *   热路径（PID_Calc / PID_CalcDt）：全程纯整数（Q16.16 + int64_t 中间量）。
 *     在 ISR 中每 5ms/20ms 调用，耗时 < 2µs（vs 浮点版 ~31µs）。
 *
 * ── 参数约定 ──
 *   kp / ki / kd ：float（如 0.1f, 0.0004f, 44.0f），内部自动转整数
 *   dt           ：毫秒整数（如 5 → 5ms, 20 → 20ms）
 *   Target / Actual / error / deadband / integral_separation / Out_max：
 *     自然单位整数，和实际物理量一致
 *   Integral_max ：I_out 最大贡献值（自然单位，和 output 同量纲）
 * ══════════════════════════════════════════════════════════════════════ */

/* ── 内部定点格式（用户无需关心）── */
#define PID_Q     16
#define PID_SCALE ((int32_t)(1 << PID_Q))   /* 65536 */

/* ── 毫秒 → Q16.16 秒（内部使用）── */
#define PID_DT_MS_TO_Q16(m) ((int32_t)((((int64_t)(m) << PID_Q) + 500) / 1000))

#define PID_ENABLE  (1U)
#define PID_DISABLE (0U)

/* ── 模式枚举 ── */
typedef enum
{
    PID_MODE_POSITION = 0,      /* 位置式 PID */
    PID_MODE_INCREMENTAL = 1    /* 增量式 PID */
} PID_Mode_t;

/* ── PID 控制器结构体 ── */
typedef struct
{
    /* ── PID 系数（Q16.16，内部使用）── */
    int32_t kp;
    int32_t ki;
    int32_t kd;

    /* ── 目标 / 实际 / 输出（自然单位）── */
    int32_t Target;
    int32_t Actual;
    int32_t output;

    /* ── P / I / D 分项输出（自然单位，在线调试用）── */
    int32_t P_out;
    int32_t D_out;
    int32_t I_out;

    /* ── 误差历史（自然单位）── */
    int32_t error0;             /* e(k)   当前误差                 */
    int32_t error1;             /* e(k-1) 上一拍误差               */
    int32_t error2;             /* e(k-2) 上上拍误差               */

    /* ── 积分累加器（Q16.16，64-bit 防长期运行溢出）── */
    int64_t error_sum;

    /* ── 限幅 ── */
    int32_t Integral_max_user;  /* 用户设置：I_out 上限（自然单位） */
    int64_t Integral_max_q16;   /* 内部：error_sum 上限（Q16.16）   */
    int32_t Out_max;            /* 输出上限（自然单位）              */

    /* ── 采样周期（Q16.16 秒，内部使用）── */
    int32_t dt;

    /* ── 死区与积分分离阈值（自然单位，和误差同量纲）── */
    int32_t deadband;               /* |error| <= deadband → error=0    */
    int32_t integral_separation;    /* |error| > 阈值 → 暂停积分        */

    /* ── 微分低通滤波器（Q16.16）── */
    int32_t d_lpf_alpha;        /* 滤波系数，范围 [0, 65536]            */
    int32_t d_filter_state;     /* 滤波器内部状态                       */

    /* ── 控制开关 ── */
    uint8_t enable;             /* PID 使能开关                         */
    uint8_t anti_windup;        /* 抗积分饱和开关                       */
    uint8_t mode;               /* 位置式 / 增量式                      */
} PID_TypeDef;

/* ── 串级 PID（外环 + 内环）── */
typedef struct
{
    PID_TypeDef* outer;
    PID_TypeDef* inner;
} PID_Cascade_t;

/* ── 编码器双轮速度环（左右独立 PID，同参数）── */
typedef struct
{
    PID_TypeDef left;
    PID_TypeDef right;
} PID_EncoderSpeed_t;

/* ══════════════════════════════════════════════════════════════════════
 * API 函数声明
 * ══════════════════════════════════════════════════════════════════════ */

/* ── 通用工具 ── */
int32_t Limit_Output(int32_t value, int32_t max);

/* ── 初始化与配置（参数说明见文件头）── */

void PID_Init(PID_TypeDef* pid);
/* 结构体清零 + 默认参数（dt=2ms，alpha=0.35，位置式）               */

void PID_Init_WithLimit(PID_TypeDef* pid, int32_t Integral_max, int32_t Out_max);
/* Integral_max: I_out 最大贡献值（自然单位）                         */
/* Out_max:      总输出上限（自然单位）                                */

void PID_Reset(PID_TypeDef* pid);
/* 清零运行时状态（error 历史 / 积分 / 微分滤波），保留参数与配置     */

void PID_Enable(PID_TypeDef* pid, uint8_t enable);
/* 关闭时自动清状态，防止再次使能时瞬态冲击                           */

void PID_SetMode(PID_TypeDef* pid, PID_Mode_t mode);
/* 切换位置式 / 增量式，自动清状态                                    */

/* ── 参数设置（全部使用用户友好尺度）── */

void PID_SetTarget(PID_TypeDef* pid, int32_t target);
/* 设置目标值（自然单位）                                             */

void PID_SetDeadband(PID_TypeDef* pid, int32_t deadband);
/* 设置死区（自然单位，和误差同量纲）                                  */

void PID_SetSampleTime(PID_TypeDef* pid, uint16_t dt_ms);
/* 设置采样周期（毫秒）。例：5ms 周期 → PID_SetSampleTime(&pid, 5)   */

void PID_SetOutputLimit(PID_TypeDef* pid, int32_t out_max);
/* 设置输出限幅（自然单位，和 output 同量纲）                          */

void PID_SetIntegralLimit(PID_TypeDef* pid, int32_t max_i_out);
/* 设置 I_out 最大贡献值（自然单位）。需在 Set_PID 之后调用           */

void PID_SetIntegralSeparation(PID_TypeDef* pid, int32_t threshold);
/* 设置积分分离阈值（自然单位）。|error| > threshold → 暂停积分       */

void PID_SetDerivativeLPF(PID_TypeDef* pid, int32_t alpha);
/* 设置微分低通系数。范围 [0, 65536]，0 = 无滤波，65536 = 最强       */
/* 常用值：0.35×65536≈22938（较小滤波），0.1×65536≈6554（强滤波）   */

void PID_SetAntiWindup(PID_TypeDef* pid, uint8_t enable);
/* 抗积分饱和开关：输出饱和且误差继续同向 → 撤销当次积分              */

void Set_PID(PID_TypeDef* pid, float kp, float ki, float kd);
/* 设置 PID 三参数（浮点，Init 阶段一次性转为内部 Q16.16）。           */
/* 例：Set_PID(&pid, 0.1f, 0.0004f, 0.005f);                          */
/* 调用后自动重算内部积分限幅（基于 ki 和 Integral_max_user）         */

/* ── PID 计算（热路径，纯整数）── */

int32_t PID_Calc(PID_TypeDef* pid, int32_t actual);
/* 使用默认 dt 做一次 PID 计算，返回控制量（自然单位）                */

int32_t PID_CalcDt(PID_TypeDef* pid, int32_t actual, int32_t dt_q16);
/* 使用显式 dt（Q16.16）做一次 PID 计算。周期不固定时使用             */

/* ── 串级 PID ── */

void PID_Cascade_Init(PID_Cascade_t* cascade, PID_TypeDef* outer, PID_TypeDef* inner);
int32_t PID_Cascade_Calc(PID_Cascade_t* cascade,
                         int32_t outer_actual, int32_t inner_actual,
                         int32_t outer_dt_q16, int32_t inner_dt_q16);

/* ── 编码器速度环 ── */

void PID_EncoderSpeed_Init(PID_EncoderSpeed_t* speed);
/* 左右轮独立 PID 初始化，Out_max=400（TB6612），I_max 默认很大       */

void PID_EncoderSpeed_Set(PID_EncoderSpeed_t* speed,
                          float kp, float ki, float kd,
                          int32_t target);
/* 设置速度环参数与目标（kp/ki/kd 浮点，Init 阶段一次性转 Q16.16）。   */
/* 例：PID_EncoderSpeed_Set(&sp, 4.0f, 44.0f, 0.0f, 15);             */

void PID_EncoderSpeed_Control(PID_EncoderSpeed_t* speed,
                              int32_t actual_left, int32_t actual_right,
                              int32_t* out_left, int32_t* out_right);
/* 一次速度环控制计算，输出左右轮控制量（自然单位）                    */

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
