#ifndef __TASKS_H
#define __TASKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════════════════
 * 任务链调度框架
 *
 * 按键协议：
 *   KEY1 — 启动（待机时按下，锁存当前选中任务号并启动）
 *   KEY2 — 选择任务（循环切换 1→2→3→4→1，由 KEY.c 维护）
 *   KEY3 — 急停（运行中按下 → 停车 + 全部 PID 清零）
 *   KEY4 — 保留
 *
 * Task_Run 在 TIMG0 ISR 20ms 插槽调用。
 * 启动瞬间锁存 s_task_select → s_task_active，运行中 KEY2 不影响当前任务。
 * ══════════════════════════════════════════════════════════════════════ */

void Task_Run(void);
/*
 * 急停：停车 + 复位全部 PID + 清除标志。
 *
 * @param brakeDuty  反向刹车占空比（0 = 直接停，>0 = 反向刹车 200ms 后归零）
 *                   Task_2 用 2000（防惯性滑动），Task_3/4 用 0（已平滑减速）
 */
void Task_Stop(int16_t brakeDuty);
uint8_t Task_IsRunning(void);      /* 1 = 运行中                           */
uint8_t Task_GetSelect(void);      /* KEY2 当前选中任务号 (1-6)            */
uint8_t Task_GetActive(void);      /* 正在运行的任务号，待机时为 0         */
uint8_t Task_GetPos(void);         /* 当前位置：1-6 段中，7=完成           */

/* ── Task_2 圈时接口（供 OLED 显示）── */
uint32_t Task_2_GetLapTime(void);   /* 返回圈时（秒），运行中=实时，完成=冻结，其他=0 */

#ifdef __cplusplus
}
#endif

#endif /* __TASKS_H */
