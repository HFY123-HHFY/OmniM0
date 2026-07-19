#include "Detect.h"
#include "Control/Control.h"  /* g_graySensor */

/* ══════════════════════════════════════════════════════════════════════
 * 灰度状态转换检测 — 互斥双检测器 + LOCKOUT + 消抖
 *
 * 核心原则：车在地图上只有一种状态——要么在黑线上，要么在白底上。
 * 因此 GrayDetect_EnterLine 和 GrayDetect_ExitLine 不可能同时为 1。
 *
 * 互斥保证：
 *   - 每个检测器触发后进入 LOCKOUT，等对方条件满足 + 消抖才重新武装。
 *   - EnterLine LOCKOUT → 连续 N 次全白 → IDLE（防边界噪点误重武装）
 *   - ExitLine  LOCKOUT → 连续 N 次见黑 → ARMED
 *   - 自然互锁：一个在 LOCKOUT 时，另一个在等待/确认中。
 *
 * 调用频率：5ms（TIMG0 ISR 内）。
 * 确认延迟：3×5ms = 15ms。
 * ══════════════════════════════════════════════════════════════════════ */

#define GRAY_DETECT_CONFIRM_CNT  3U   /* 连续确认次数 */

/* ── 内部状态枚举 ── */
typedef enum {
    GRAY_DET_IDLE = 0U,
    GRAY_DET_ARMED,         /* ExitLine 专用：已见黑，等全白            */
    GRAY_DET_CONFIRM_1,
    GRAY_DET_CONFIRM_2,
    GRAY_DET_CONFIRM_3,
    GRAY_DET_FIRED,
    GRAY_DET_LOCKOUT        /* 触发后锁定，消抖后才重新武装            */
} GrayDetect_State_t;

/*
 * GrayDetect_EnterLine — 检测"白底 → 黑线"转换。
 *
 * 状态机：
 *   IDLE      — 等见黑 → CONFIRM_1/2/3
 *   CONFIRM   — 持续见黑 → FIRED；见白退回 IDLE（噪点）
 *   FIRED     — → LOCKOUT，返回 1
 *   LOCKOUT   — 连续 3 次全白（消抖）→ IDLE（重新武装）
 *
 * 触发条件：8 路任意一路见黑（digital_bits[i] == 0）。
 * 返回 1：检测到入线事件（每个入线周期仅触发一次）。
 */
uint8_t GrayDetect_EnterLine(void)
{
    static GrayDetect_State_t s_state   = GRAY_DET_IDLE;
    static uint8_t            s_confirm = 0U;
    uint8_t i;
    uint8_t any_black = 0U;

    for (i = 0U; i < 8U; ++i)
    {
        if (g_graySensor.digital_bits[i] == 0U) { any_black = 1U; break; }
    }

    switch (s_state)
    {
    case GRAY_DET_IDLE:
        if (any_black != 0U)
        {
            s_state   = GRAY_DET_CONFIRM_1;
            s_confirm = 1U;
        }
        break;

    case GRAY_DET_CONFIRM_1:
    case GRAY_DET_CONFIRM_2:
    case GRAY_DET_CONFIRM_3:
        if (any_black != 0U)
        {
            s_confirm++;
            if (s_confirm >= GRAY_DETECT_CONFIRM_CNT)
            {
                s_state   = GRAY_DET_FIRED;
                s_confirm = 0U;
            }
            else
            {
                s_state = (GrayDetect_State_t)((uint8_t)s_state + 1U);
            }
        }
        else
        {
            s_state   = GRAY_DET_IDLE;
            s_confirm = 0U;
        }
        break;

    case GRAY_DET_FIRED:
        s_state   = GRAY_DET_LOCKOUT;
        s_confirm = 0U;              /* s_confirm 复用为 LOCKOUT 消抖计数器 */
        return 1U;

    case GRAY_DET_LOCKOUT:
        /*
         * 连续 N 次全白才重新武装，防止边界噪点（刚离线时
         * 传感器可能短暂见黑）让 EnterLine 立即重新触发，
         * 从而在 ISR 互斥清零中把刚置位的 exit_fired 干掉。
         */
        if (any_black == 0U)
        {
            s_confirm++;
            if (s_confirm >= GRAY_DETECT_CONFIRM_CNT)
            {
                s_state   = GRAY_DET_IDLE;
                s_confirm = 0U;
            }
        }
        else
        {
            s_confirm = 0U;          /* 见黑 → 重置消抖计数 */
        }
        break;

    default:
        s_state = GRAY_DET_IDLE;
        break;
    }

    return 0U;
}

/*
 * GrayDetect_ExitLine — 检测"黑线 → 白底"转换。
 *
 * 状态机：
 *   IDLE      — 先等见黑 → ARMED（上电在白底不会误触发）
 *   ARMED     — 等到全白 → CONFIRM_1/2/3
 *   CONFIRM   — 持续全白 → FIRED；又见黑退回 ARMED（噪点）
 *   FIRED     — → LOCKOUT，返回 1
 *   LOCKOUT   — 连续 3 次见黑（消抖）→ ARMED（重新武装）
 *
 * 触发条件：先见过黑，然后 8 路全白。
 * 返回 1：检测到离线事件（每个离线周期仅触发一次）。
 */
uint8_t GrayDetect_ExitLine(void)
{
    static GrayDetect_State_t s_state   = GRAY_DET_IDLE;
    static uint8_t            s_confirm = 0U;
    uint8_t i;
    uint8_t any_black = 0U;

    for (i = 0U; i < 8U; ++i)
    {
        if (g_graySensor.digital_bits[i] == 0U) { any_black = 1U; break; }
    }

    switch (s_state)
    {
    case GRAY_DET_IDLE:
        if (any_black != 0U)
        {
            s_state = GRAY_DET_ARMED;
        }
        break;

    case GRAY_DET_ARMED:
        if (any_black == 0U)
        {
            s_state   = GRAY_DET_CONFIRM_1;
            s_confirm = 1U;
        }
        break;

    case GRAY_DET_CONFIRM_1:
    case GRAY_DET_CONFIRM_2:
    case GRAY_DET_CONFIRM_3:
        if (any_black == 0U)
        {
            s_confirm++;
            if (s_confirm >= GRAY_DETECT_CONFIRM_CNT)
            {
                s_state   = GRAY_DET_FIRED;
                s_confirm = 0U;
            }
            else
            {
                s_state = (GrayDetect_State_t)((uint8_t)s_state + 1U);
            }
        }
        else
        {
            s_state   = GRAY_DET_ARMED;
            s_confirm = 0U;
        }
        break;

    case GRAY_DET_FIRED:
        s_state   = GRAY_DET_LOCKOUT;
        s_confirm = 0U;              /* s_confirm 复用为 LOCKOUT 消抖计数器 */
        return 1U;

    case GRAY_DET_LOCKOUT:
        /*
         * 连续 N 次见黑才重新武装，与 EnterLine 的 LOCKOUT 消抖对应。
         * 防止刚入线时传感器噪点（短暂全白）让 ExitLine 误重武装。
         */
        if (any_black != 0U)
        {
            s_confirm++;
            if (s_confirm >= GRAY_DETECT_CONFIRM_CNT)
            {
                s_state   = GRAY_DET_ARMED;
                s_confirm = 0U;
            }
        }
        else
        {
            s_confirm = 0U;          /* 见白 → 重置消抖计数 */
        }
        break;

    default:
        s_state = GRAY_DET_IDLE;
        break;
    }

    return 0U;
}
