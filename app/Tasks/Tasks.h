#ifndef __TASKS_H
#define __TASKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 灰度事件标志位（5ms ISR 置位，任务消费清零）── */
/* 检测器内部 LOCKOUT 保证每次入/离线只触发一次，不会反复置位 */
extern volatile uint8_t s_gray_enter_fired;   /* 入线事件：1 = 待消费 */
extern volatile uint8_t s_gray_exit_fired;    /* 出线事件：1 = 待消费 */

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

void    Task_Run(void);
void    Task_Stop(void);           /* 急停：停车 + 复位全部 PID + 清灰度标志 */
uint8_t Task_IsRunning(void);      /* 1 = 运行中                           */
uint8_t Task_GetSelect(void);      /* KEY2 当前选中任务号 (1-4)            */
uint8_t Task_GetActive(void);      /* 正在运行的任务号，待机时为 0         */
uint8_t Task_GetPos(void);         /* 当前位置：1-4 段中，5=完成           */

/* ── 任务函数（空壳，待开发）── */
void Task_1(void);
void Task_2(void);
void Task_3(void);
void Task_4(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASKS_H */
