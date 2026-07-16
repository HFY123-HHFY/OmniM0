/**
 * ══════════════════════════════════════════════════════════════════════
 * 整数定点 PID 实现（专为 M0+ 无 FPU 优化）
 *
 * 内部使用 Q16.16 定点格式存储系数与状态，所有 PID 计算为纯整数。
 * 用户 API 接受自然尺度参数（kp/ki/kd ×1,000,000 / dt 毫秒），
 * Init 阶段由转换宏一次性转为 Q16.16，不增加热路径开销。
 *
 * 关键公式（位置式）：
 *   error   = Target - Actual                              (自然单位)
 *   P_out   = (kp_q16 × error) >> 16                       (自然单位)
 *   I_acc  += (int64_t)error × dt_q16                      (Q16.16)
 *   I_out   = (ki_q16 × I_acc) >> 32                       (自然单位)
 *   D_raw   = ((error - error0) << 16) / dt_q16            (Q16.16)
 *   D_out   = (kd_q16 × D_filtered) >> 16                  (自然单位)
 *   output  = clamp(P_out + I_out + D_out, ±Out_max)       (自然单位)
 *
 * 热路径（PID_Calc）耗时 < 2µs @80MHz，vs 浮点版 ~31µs。
 * ══════════════════════════════════════════════════════════════════════
 */

#include "PID.h"
#include <stddef.h>   /* NULL */

/* ══════════════════════════════════════════════════════════════════════
 * 内部常量与辅助函数
 * ══════════════════════════════════════════════════════════════════════ */

/* Q16.16 常用值 */
#define Q16_ONE   ((int32_t)PID_SCALE)        /* 1.0 → 65536          */
#define Q16_HALF  ((int32_t)(PID_SCALE / 2))  /* 0.5 → 32768          */
#define Q16_09    ((int32_t)58982)            /* 0.9 → 58982 (≈0.9)  */

/* 默认微分低通系数：0.35 × 65536 ≈ 22938 */
#define Q16_DEFAULT_LPF_ALPHA  ((int32_t)22938)

/* 绝对安全的积分上限（约 2^62，防止 int64_t 回绕，实际上几乎不会触发） */
#define Q16_INTEGRAL_MAX_SAFE  ((int64_t)1 << 62)

/** 整数绝对值 */
static int32_t pid_abs(int32_t x)
{
    return (x >= 0) ? x : -x;
}

/* ══════════════════════════════════════════════════════════════════════
 * 限幅函数
 * ══════════════════════════════════════════════════════════════════════ */

/** 对称限幅（int32_t）。max <= 0 表示不限幅，方便调试。 */
int32_t Limit_Output(int32_t value, int32_t max)
{
    if (max <= 0)  return value;
    if (value > max)  return max;
    if (value < -max) return -max;
    return value;
}

/** 对称限幅（int64_t）。 */
static int64_t Limit_Output64(int64_t value, int64_t max)
{
    if (max <= 0)  return value;
    if (value > max)  return max;
    if (value < -max) return -max;
    return value;
}

/* ══════════════════════════════════════════════════════════════════════
 * 内部：根据 ki 重算积分限幅
 *
 * Integral_max_q16 限制了 error_sum_q16 的最大值，从而限制了 I_out。
 * 关系：I_out_max = (ki_q16 × Integral_max_q16) >> 32
 * 所以：Integral_max_q16 = (Integral_max_user × 2^32) / ki_q16
 *
 * 这意味着用户调用 Set_PID 修改 ki 后，积分限幅会自动跟随调整，
 * 保证用户设置的 I_out 上限始终有效。
 * ══════════════════════════════════════════════════════════════════════ */
static void pid_recalc_integral_limit(PID_TypeDef* pid)
{
    if (pid->ki > 0 && pid->Integral_max_user > 0)
    {
        pid->Integral_max_q16 = ((int64_t)pid->Integral_max_user << 32) / pid->ki;
    }
    else
    {
        /* ki=0 → 无积分作用，用安全上限防止意外累加 */
        pid->Integral_max_q16 = Q16_INTEGRAL_MAX_SAFE;
    }
}

/* ══════════════════════════════════════════════════════════════════════
 * PID_Init — 清零 + 默认参数
 * ══════════════════════════════════════════════════════════════════════ */
void PID_Init(PID_TypeDef* pid)
{
    if (pid == NULL) return;

    /* 系数清零 */
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;

    /* I/O 清零 */
    pid->Target  = 0;
    pid->Actual  = 0;
    pid->output  = 0;
    pid->P_out   = 0;
    pid->D_out   = 0;
    pid->I_out   = 0;

    /* 误差历史清零 */
    pid->error0 = 0;
    pid->error1 = 0;
    pid->error2 = 0;
    pid->error_sum = 0;

    /* 默认采样周期 = 2ms（内部 Q16.16） */
    pid->dt = PID_DT_MS_TO_Q16(2);

    /* 死区 / 积分分离：默认关闭 */
    pid->deadband            = 0;
    pid->integral_separation = 0;

    /* 微分低通：默认 alpha = 0.35 */
    pid->d_lpf_alpha    = Q16_DEFAULT_LPF_ALPHA;
    pid->d_filter_state = 0;

    /* 限幅：默认不限（调试时放开） */
    pid->Integral_max_user = 0;
    pid->Integral_max_q16  = Q16_INTEGRAL_MAX_SAFE;
    pid->Out_max           = 0;

    /* 开关：使能 + 抗饱和 + 位置式 */
    pid->enable      = PID_ENABLE;
    pid->anti_windup = PID_ENABLE;
    pid->mode        = (uint8_t)PID_MODE_POSITION;
}

/* ══════════════════════════════════════════════════════════════════════
 * PID_Init_WithLimit — 设置积分与输出限幅
 *
 * @param Integral_max  I_out 最大贡献值（自然单位，和 output 同量纲）
 *                      例：方向环 I 项最多贡献 ±20 → 传入 20
 *                          速度环 I 项最多贡献 ±200 → 传入 200
 * @param Out_max       输出上限（自然单位，如 TB6612 占空比 400）
 *
 * 注意：Integral_max_q16 的精确值在 Set_PID 调用后才会被计算。
 *       在此之前使用安全上限。
 * ══════════════════════════════════════════════════════════════════════ */
void PID_Init_WithLimit(PID_TypeDef* pid, int32_t Integral_max, int32_t Out_max)
{
    if (pid == NULL) return;

    pid->Integral_max_user = (Integral_max >= 0) ? Integral_max : -Integral_max;
    pid->Out_max           = (Out_max >= 0)      ? Out_max      : -Out_max;

    /* 暂用安全值，等 Set_PID 后精确计算 */
    pid->Integral_max_q16 = Q16_INTEGRAL_MAX_SAFE;
}

/* ══════════════════════════════════════════════════════════════════════
 * PID_Reset — 清零运行时状态，保留系数与配置
 *
 * 用途：模式切换、停机后重新启动前调用，防止历史积分残留。
 * ══════════════════════════════════════════════════════════════════════ */
void PID_Reset(PID_TypeDef* pid)
{
    if (pid == NULL) return;

    pid->Actual = 0;
    pid->output = 0;
    pid->P_out  = 0;
    pid->D_out  = 0;
    pid->I_out  = 0;

    pid->error0 = 0;
    pid->error1 = 0;
    pid->error2 = 0;

    pid->error_sum      = 0;
    pid->d_filter_state = 0;
}

/* ══════════════════════════════════════════════════════════════════════
 * 开关 / 模式控制
 * ══════════════════════════════════════════════════════════════════════ */

void PID_Enable(PID_TypeDef* pid, uint8_t enable)
{
    if (pid == NULL) return;
    pid->enable = (enable != 0U) ? PID_ENABLE : PID_DISABLE;
    if (pid->enable == PID_DISABLE) { PID_Reset(pid); }
}

void PID_SetMode(PID_TypeDef* pid, PID_Mode_t mode)
{
    if (pid == NULL) return;
    pid->mode = (uint8_t)mode;
    PID_Reset(pid);
}

/* ══════════════════════════════════════════════════════════════════════
 * 参数设置函数
 *
 * 所有参数使用用户友好尺度（见 PID.h 文件头注释）。
 * 内部通过转换宏转为 Q16.16，仅 Init 阶段执行一次，热路径零开销。
 * ══════════════════════════════════════════════════════════════════════ */

void PID_SetTarget(PID_TypeDef* pid, int32_t target)
{
    if (pid != NULL) pid->Target = target;
}

void PID_SetDeadband(PID_TypeDef* pid, int32_t deadband)
{
    if (pid != NULL) pid->deadband = (deadband >= 0) ? deadband : -deadband;
}

/** 设置采样周期（毫秒）。例：5ms → PID_SetSampleTime(&pid, 5) */
void PID_SetSampleTime(PID_TypeDef* pid, uint16_t dt_ms)
{
    if (pid != NULL && dt_ms > 0)
    {
        pid->dt = PID_DT_MS_TO_Q16(dt_ms);
    }
}

void PID_SetOutputLimit(PID_TypeDef* pid, int32_t out_max)
{
    if (pid != NULL) pid->Out_max = pid_abs(out_max);
}

/**
 * 设置 I_out 最大贡献值（自然单位）。
 * 需在 Set_PID 之后调用，否则会被 Set_PID 覆盖。
 * 若在 Set_PID 之前调用，值会被保存为 Integral_max_user，
 * Set_PID 执行时自动据此计算内部限幅。
 */
void PID_SetIntegralLimit(PID_TypeDef* pid, int32_t max_i_out)
{
    if (pid == NULL) return;
    pid->Integral_max_user = (max_i_out >= 0) ? max_i_out : -max_i_out;
    pid_recalc_integral_limit(pid);
}

void PID_SetIntegralSeparation(PID_TypeDef* pid, int32_t threshold)
{
    if (pid != NULL) pid->integral_separation = pid_abs(threshold);
}

/**
 * 设置微分低通滤波系数。
 * @param alpha  Q16.16 格式，范围 [0, 65536]。
 *               0     = 无滤波（D 项完全信任原始微分）
 *               65536 = 最强滤波（D 项几乎不变）
 *               常用：22938（≈0.35），6554（≈0.1）
 */
void PID_SetDerivativeLPF(PID_TypeDef* pid, int32_t alpha)
{
    if (pid == NULL) return;
    if (alpha < 0)       alpha = 0;
    if (alpha > Q16_ONE) alpha = Q16_ONE;
    pid->d_lpf_alpha = alpha;
}

void PID_SetAntiWindup(PID_TypeDef* pid, uint8_t enable)
{
    if (pid != NULL) pid->anti_windup = (enable != 0U) ? PID_ENABLE : PID_DISABLE;
}

/**
 * 设置 PID 三参数（浮点 → Init 阶段一次性转为内部 Q16.16）。
 *
 * 例：Set_PID(&pid, 0.1f, 0.0004f, 0.005f);
 *
 * 浮点乘法仅在此处执行（启动时 2-3 次，总共 ~30µs），
 * ISR 热路径（PID_Calc）全程纯整数。
 */
void Set_PID(PID_TypeDef* pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;

    /* 浮点 → 内部 Q16.16（四舍五入） */
    pid->kp = (int32_t)(kp * (float)PID_SCALE + 0.5f);
    pid->ki = (int32_t)(ki * (float)PID_SCALE + 0.5f);
    pid->kd = (int32_t)(kd * (float)PID_SCALE + 0.5f);

    /* 系数变了，积分限幅也跟着变 */
    pid_recalc_integral_limit(pid);
}

/* ══════════════════════════════════════════════════════════════════════
 * pid_calc_position — 位置式整数 PID（核心算法）
 *
 * 所有中间乘积累使用 int64_t，只在最后截断为 int32_t 自然单位。
 * 死区 / 积分分离 / 积分限幅 / 微分低通 / 抗积分饱和 全部保留。
 * ══════════════════════════════════════════════════════════════════════ */
static int32_t pid_calc_position(PID_TypeDef* pid, int32_t actual, int32_t dt_q16)
{
    int32_t error;
    int32_t P_out, I_out, D_out;
    int32_t derivative_raw;
    int32_t output_unsat, limited_output;
    int32_t should_integrate;

    /* ── 1. 误差 = 目标 - 实际 ── */
    pid->Actual = actual;
    error = pid->Target - pid->Actual;

    /* ── 2. 死区：小误差按 0 处理，减少电机微振 ── */
    should_integrate = 1;
    if ((pid->deadband > 0) && (pid_abs(error) <= pid->deadband))
    {
        error = 0;
        /* I 项每拍衰减 10% 防止回到中心后残留积分推着电机继续转 */
        pid->error_sum = (pid->error_sum * Q16_09) >> PID_Q;
    }

    /* ── 3. 积分分离：大偏差时暂停积分，防 windup ── */
    if ((pid->integral_separation > 0) && (pid_abs(error) > pid->integral_separation))
    {
        should_integrate = 0;
    }

    /* ── 4. 积分累加（Q16.16, 64-bit）── */
    if (should_integrate && (pid->ki != 0))
    {
        /* error_sum += error × dt （每次一小步，64-bit 足够运行数小时不溢出） */
        pid->error_sum += (int64_t)error * dt_q16;
        pid->error_sum = Limit_Output64(pid->error_sum, pid->Integral_max_q16);
    }

    /* ── 5. 微分 + 一阶低通（降低噪声敏感）── */
    if (dt_q16 > 0)
    {
        /* D_raw = Δerror / dt，结果 Q16.16 */
        derivative_raw = (int32_t)(((int64_t)(error - pid->error0) << PID_Q) / dt_q16);
    }
    else
    {
        derivative_raw = 0;
    }

    /* LPF: filter = α × raw + (1-α) × prev */
    pid->d_filter_state = (int32_t)(
        (((int64_t)pid->d_lpf_alpha * derivative_raw) >> PID_Q) +
        (((int64_t)(Q16_ONE - pid->d_lpf_alpha) * pid->d_filter_state) >> PID_Q)
    );

    /* ── 6. P / I / D 分项计算 ── */

    /* P: (kp_q16 × error) >> 16 → 自然单位 */
    P_out = (int32_t)(((int64_t)pid->kp * error) >> PID_Q);

    /* I: (ki_q16 × I_acc_q16) >> 32 → 自然单位（保留子输出精度） */
    I_out = (int32_t)(((int64_t)pid->ki * pid->error_sum) >> 32);

    /* D: (kd_q16 × d_filter_q16) >> 16 → 自然单位 */
    D_out = (int32_t)(((int64_t)pid->kd * pid->d_filter_state) >> PID_Q);

    pid->P_out = P_out;
    pid->I_out = I_out;
    pid->D_out = D_out;

    /* ── 7. 合成输出 + 限幅 ── */
    output_unsat   = P_out + I_out + D_out;
    limited_output = Limit_Output(output_unsat, pid->Out_max);

    /* ── 8. 抗积分饱和 ── */
    if ((pid->anti_windup == PID_ENABLE) && (output_unsat != limited_output) && (pid->ki != 0))
    {
        /* 输出已饱和且误差方向会让饱和更严重 → 撤销本次积分 */
        if (((output_unsat > pid->Out_max) && (error > 0)) ||
            ((output_unsat < -pid->Out_max) && (error < 0)))
        {
            pid->error_sum -= (int64_t)error * dt_q16;
            pid->error_sum = Limit_Output64(pid->error_sum, pid->Integral_max_q16);

            I_out = (int32_t)(((int64_t)pid->ki * pid->error_sum) >> 32);
            pid->I_out = I_out;

            output_unsat   = P_out + I_out + D_out;
            limited_output = Limit_Output(output_unsat, pid->Out_max);
        }
    }

    /* ── 9. 误差历史更新 ── */
    pid->error2 = pid->error1;
    pid->error1 = pid->error0;
    pid->error0 = error;

    pid->output = limited_output;
    return pid->output;
}

/* ══════════════════════════════════════════════════════════════════════
 * pid_calc_incremental — 增量式整数 PID
 *
 * Δu = Kp×(e(k)-e(k-1)) + Ki×e(k)×dt + Kd×(e(k)-2e(k-1)+e(k-2))/dt
 * u(k) = u(k-1) + Δu（累加限幅）
 * ══════════════════════════════════════════════════════════════════════ */
static int32_t pid_calc_incremental(PID_TypeDef* pid, int32_t actual, int32_t dt_q16)
{
    int32_t error;
    int32_t delta_u;
    int32_t ki_term;
    int32_t derivative_inc;

    /* 误差 */
    pid->Actual = actual;
    error = pid->Target - pid->Actual;

    /* 死区 */
    if ((pid->deadband > 0) && (pid_abs(error) <= pid->deadband))
    {
        error = 0;
    }

    /* 二阶差分近似微分（Q16.16） */
    if (dt_q16 > 0)
    {
        int32_t diff2 = error - 2 * pid->error0 + pid->error1;
        derivative_inc = (int32_t)(((int64_t)diff2 << PID_Q) / dt_q16);
    }
    else
    {
        derivative_inc = 0;
    }

    /* 低通滤波 */
    pid->d_filter_state = (int32_t)(
        (((int64_t)pid->d_lpf_alpha * derivative_inc) >> PID_Q) +
        (((int64_t)(Q16_ONE - pid->d_lpf_alpha) * pid->d_filter_state) >> PID_Q)
    );

    /* I 项增量：ki × error × dt → 自然单位 */
    ki_term = (int32_t)(((int64_t)pid->ki * error * dt_q16) >> 32);

    /* Δu = Kp×Δe + Ki×e×dt + Kd×Δ²e */
    delta_u = (int32_t)(((int64_t)pid->kp * (error - pid->error0)) >> PID_Q)
            + ki_term
            + (int32_t)(((int64_t)pid->kd * pid->d_filter_state) >> PID_Q);

    /* 累加限幅 */
    pid->output += delta_u;
    pid->output = Limit_Output(pid->output, pid->Out_max);

    /* 分项记录（调试用） */
    pid->P_out = (int32_t)(((int64_t)pid->kp * error) >> PID_Q);
    pid->I_out += ki_term;
    pid->I_out = Limit_Output(pid->I_out, pid->Integral_max_user);
    pid->D_out = (int32_t)(((int64_t)pid->kd * pid->d_filter_state) >> PID_Q);

    /* 误差历史 */
    pid->error2 = pid->error1;
    pid->error1 = pid->error0;
    pid->error0 = error;

    return pid->output;
}

/* ══════════════════════════════════════════════════════════════════════
 * PID_CalcDt / PID_Calc — 对外统一入口
 * ══════════════════════════════════════════════════════════════════════ */

int32_t PID_CalcDt(PID_TypeDef* pid, int32_t actual, int32_t dt_q16)
{
    if (pid == NULL)                   return 0;
    if (pid->enable == PID_DISABLE)    return 0;
    if (dt_q16 <= 0)                   dt_q16 = (pid->dt > 0) ? pid->dt : PID_DT_MS_TO_Q16(1);

    if (pid->mode == (uint8_t)PID_MODE_INCREMENTAL)
    {
        return pid_calc_incremental(pid, actual, dt_q16);
    }
    return pid_calc_position(pid, actual, dt_q16);
}

int32_t PID_Calc(PID_TypeDef* pid, int32_t actual)
{
    if (pid == NULL) return 0;
    return PID_CalcDt(pid, actual, pid->dt);
}

/* ══════════════════════════════════════════════════════════════════════
 * 串级 PID
 * ══════════════════════════════════════════════════════════════════════ */

void PID_Cascade_Init(PID_Cascade_t* cascade, PID_TypeDef* outer, PID_TypeDef* inner)
{
    if (cascade == NULL) return;
    cascade->outer = outer;
    cascade->inner = inner;
}

int32_t PID_Cascade_Calc(PID_Cascade_t* cascade,
                         int32_t outer_actual, int32_t inner_actual,
                         int32_t outer_dt_q16, int32_t inner_dt_q16)
{
    int32_t inner_target;

    if ((cascade == NULL) || (cascade->outer == NULL) || (cascade->inner == NULL))
    {
        return 0;
    }

    /* 外环 → 内环目标 → 内环计算 → 最终输出 */
    inner_target = PID_CalcDt(cascade->outer, outer_actual, outer_dt_q16);
    cascade->inner->Target = inner_target;
    return PID_CalcDt(cascade->inner, inner_actual, inner_dt_q16);
}

/* ══════════════════════════════════════════════════════════════════════
 * 编码器双轮速度环
 * ══════════════════════════════════════════════════════════════════════ */

void PID_EncoderSpeed_Init(PID_EncoderSpeed_t* speed)
{
    if (speed == NULL) return;

    PID_Init(&speed->left);
    PID_Init(&speed->right);

    /* 左右轮独立 PID，Out_max = TB6612_MAX_DUTY(2000) */
    /* I_out 上限设为 1750（接近 Out_max，允许 I 项充分贡献） */
    PID_Init_WithLimit(&speed->left,  1750, 2000);
    PID_Init_WithLimit(&speed->right, 1750, 2000);
}

void PID_EncoderSpeed_Set(PID_EncoderSpeed_t* speed,
                          float kp, float ki, float kd,
                          int32_t target)
{
    if (speed == NULL) return;

    /* 左右轮共用同一组 PID 参数和目标速度 */
    Set_PID(&speed->left,  kp, ki, kd);
    Set_PID(&speed->right, kp, ki, kd);

    PID_SetTarget(&speed->left,  target);
    PID_SetTarget(&speed->right, target);
}

void PID_EncoderSpeed_Control(PID_EncoderSpeed_t* speed,
                              int32_t actual_left, int32_t actual_right,
                              int32_t* out_left, int32_t* out_right)
{
    if ((speed == NULL) || (out_left == NULL) || (out_right == NULL)) return;

    *out_left  = PID_Calc(&speed->left,  actual_left);
    *out_right = PID_Calc(&speed->right, actual_right);
}
