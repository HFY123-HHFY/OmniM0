#ifndef __IRQ_PRIORITY_H
#define __IRQ_PRIORITY_H

/*
 * IrqPriority.h — 统一中断优先级管理
 *
 * MSPM0G3507: Cortex-M0+, __NVIC_PRIO_BITS=2, 范围 0~3（数字越小优先级越高）
 *
 * 架构：单一时基（TIMG0 1ms）+ 前后台调度
 *   - TIMG0 ISR 只做计数+置标志位（< 2µs），所有耗时任务在主循环执行
 *   - 仅一个定时器中断源，中断嵌套风险归零
 *   - 所有任务基于同一时钟源，时序完全一致
 *
 * 优先级分配：
 *  ┌──────┬──────────────────────────────────────────────────┐
 *  │  0   │ TIMG0 — 系统时基 1ms（ISR 只计数+置标志位）       │
 *  │  2   │ USART3 — JY61P 陀螺仪高速数据流（需低延迟）       │
 *  │  3   │ USART1/2/4 + MPU6050 + 缺省（调试与辅助外设）     │
 *  └──────┴──────────────────────────────────────────────────┘
 */

/* ── TIM 优先级 ── */
#define IRQ_PRIO_TIM1        0U   /* 最高：TIMG0 系统时基 1ms（前后台调度核心）  */
#define IRQ_PRIO_TIM_DEFAULT 3U   /* 缺省定时器优先级                             */

/* ── USART 优先级（按实例独立设置） ── */
#define IRQ_PRIO_USART3      2U   /* 中：JY61P 陀螺仪 115200bps 高速数据流       */
#define IRQ_PRIO_USART1      3U   /* 低：串口调试                                 */
#define IRQ_PRIO_USART2      3U   /* 低：预留                                     */
#define IRQ_PRIO_USART4      3U   /* 低：未用                                     */

/* ── 其他 ── */
#define IRQ_PRIO_MPU6050     3U   /* 低：MPU6050 外部中断（当前已注释未启用）      */
#define IRQ_PRIO_DEFAULT     3U   /* 最低：未指定中断的缺省值                      */

/* ── sub-priority（M0+ 不支持，仅为 API 签名兼容保留，值无实际作用）── */
#define IRQ_SUB_PRIO_MPU6050 0U

#endif /* __IRQ_PRIORITY_H */
