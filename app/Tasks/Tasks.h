#ifndef __TASKS_H
#define __TASKS_H

#include <stdint.h>
#include "LED.h"     /* LED_Id_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ── 灰度事件标志位（5ms ISR 置位，任务消费清零）── */
/* 检测器内部 LOCKOUT 保证每次入/离线只触发一次，不会反复置位 */
extern volatile uint8_t s_gray_enter_fired;   /* 入线事件：1 = 待消费 */
extern volatile uint8_t s_gray_exit_fired;    /* 出线事件：1 = 待消费 */

/* ── 通用声光提示（蜂鸣器 + LED2 各 200ms，非阻塞）── */
void Buzzer_Alert(uint32_t ms);

/* ══════════════════════════════════════════════════════════════════════
 * 任务链调度 — KEY1 启动/停止，KEY2 循环选择任务 (1-4)
 *
 * Task_Run 在 TIMG0 ISR 20ms 插槽调用。
 * 启动瞬间锁存任务号，运行中 KEY2 不影响当前任务。
 * ══════════════════════════════════════════════════════════════════════ */
void    Task_Run(void);
void    Task_Stop(void);           /* 停车 + PID 复位，任务内部可调用   */
uint8_t Task_IsRunning(void);      /* 1 = 运行中                        */
uint8_t Task_GetSelect(void);      /* KEY2 当前选中任务号 (1-4)         */
uint8_t Task_GetActive(void);      /* 正在运行的任务号，待机时为 0      */
uint8_t Task_GetPos(void);         /* Task_2 当前位置: 1=A,2=B,3=C,4=D,5=DONE */

/* ── 任务函数 ── */
void Task_1(void);
void Task_2(void);
void Task_3(void);
void Task_4(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASKS_H */
