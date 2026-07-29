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
    volatile bool    flag;
    volatile uint16_t tick;
    uint16_t period;
} TaskBlock;

typedef struct {
    TaskBlock buzzer_5ms;     /* 蜂鸣器/LED 非阻塞调度          */
    TaskBlock key_20ms;       /* 按键轮询（消抖在 ISR 1ms 完成）*/
    TaskBlock print_50ms;     /* 串口：调试打印                 */
    TaskBlock oled_100ms;     /* 显示：OLED 刷新                */
} TaskManager;

extern TaskManager tasks;

/* ══════════════════════════════════════════════════════════════════════
 * 非阻塞延时 — 基于 TIMG0 1ms 全局 tick
 *
 * 用法：
 *   NonBlockDelay_t buzzer_delay;
 *   NonBlockDelay_Start(&buzzer_delay, 200);   // 启动 200ms
 *   if (NonBlockDelay_IsDone(&buzzer_delay)) {  // 轮询是否到时间
 *       LED_Control(Buzzer1, LED_LOW);
 *   }
 *
 * 支持 N 个独立实例，互不干扰。uint32_t 减法天然处理 49 天溢出回绕。
 * ══════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint32_t start_tick;
    uint16_t duration_ms;
} NonBlockDelay_t;

void      NonBlockDelay_Start(NonBlockDelay_t *d, uint16_t ms);
uint8_t   NonBlockDelay_IsDone(NonBlockDelay_t *d);
uint32_t  SysTick_GetMs(void);

/* ── 串口数据包解析结果缓存（由 My_Usart 模块管理）── */
/* 全局变量 g_cam_data[] / g_cam_count 声明在 My_Usart.h */

/* ── 中断回调 ── */
void Control_Task_TIM_Callback(API_TIM_Id_t id);
void Control_Task_USART_Callback(API_USART_Id_t id);

#endif /* __CONTROL_TASK_H */
