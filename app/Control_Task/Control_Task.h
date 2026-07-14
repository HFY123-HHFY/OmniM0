#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "tim.h"
#include "usart.h"

/* ══════════════════════════════════════════════════════════════════════
 * 统一任务管理结构体 — 基于 TIMG0 1ms 时基的前后台调度器
 * ══════════════════════════════════════════════════════════════════════ */

typedef struct {
    volatile bool    flag;    /* 任务触发标志：ISR 置 true，主循环消费后清 false */
    volatile uint16_t tick;   /* 1ms 中断计数器（ISR 递增，到达 period 后归零）   */
    uint16_t period;          /* 触发周期（单位：ms）                             */
} TaskBlock;

typedef struct {
    /* ── 主循环低频任务（软实时）── */
    TaskBlock print_50ms;     /* 串口：调试数据打印（50ms）    */
    TaskBlock oled_100ms;     /* 显示：OLED 刷新（100ms）      */
} TaskManager;

extern TaskManager tasks;

/* ── 串口数据包解析缓存（USART 中断回调使用）── */
#define USART_PACKET_DATA_LEN 10U
extern int16_t USART_Packet_Data[USART_PACKET_DATA_LEN];
extern uint8_t USART_Packet_Count;

/* ── 中断回调 ── */
void Control_Task_TIM_Callback(API_TIM_Id_t id);           /* TIMG0 1ms 时基：Key_Tick + 高频任务 + TaskManager 计数 */
void Control_Task_USART_Callback(API_USART_Id_t id);       /* USART 中断回调：数据接收                              */

#endif /* __CONTROL_TASK_H */
