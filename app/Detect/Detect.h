#ifndef __DETECT_H
#define __DETECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════════════════
 * 灰度状态转换检测 — 双态判定 + 3 次确认计数
 *
 * 比单条件判断更稳，避免传感器在黑白边界反复触发。
 * 每个函数独立维护内部状态机，互不干扰。
 *
 * GrayDetect_EnterLine()：全白 → 中间 2 路见黑 → 返回 1（进入黑线）
 * GrayDetect_ExitLine() ：中间 2 路见黑 → 全白 → 返回 1（离开黑线/白底）
 *
 * 调用频率：5ms（TIMG0 ISR 内），确认计数 N=3 → 15ms 确认延迟。
 * 每个函数返回单次脉冲（触发后自动复位），后续返回 0 直至下一个事件。
 * ══════════════════════════════════════════════════════════════════════ */

uint8_t GrayDetect_EnterLine(void);  /* 白底 → 任意一路见黑 */
uint8_t GrayDetect_ExitLine(void);   /* 中间 2 路见黑 → 白底 */

#ifdef __cplusplus
}
#endif

#endif /* __DETECT_H */
