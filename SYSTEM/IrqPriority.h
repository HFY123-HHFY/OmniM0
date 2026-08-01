#ifndef __IRQ_PRIORITY_H
#define __IRQ_PRIORITY_H

/*
 * IrqPriority.h — 统一中断优先级管理
 *
 * MSPM0G3507: Cortex-M0+, __NVIC_PRIO_BITS=2, 范围 0~3（数字越小优先级越高）
 *
 * 优先级分配（按实际在用外设）：
 *  ┌──────┬──────────────────────────────────────────────────────┐
 *  │  0   │ TIMG0 — 系统时基 1ms（ISR 调度核心，不可抢占）        │
 *  │  1   │ Encoder EXTI + USART4 — 编码器脉冲 / 摄像头数据       │
 *  │  2   │ USART2 — 步进电机 Emm42 通信                         │
 *  │  3   │ USART1/3 + MPU6050 + 缺省 — 调试串口 / 未启用的外设   │
 *  └──────┴──────────────────────────────────────────────────────┘
 */

/* ── TIM 优先级 ── */
#define IRQ_PRIO_TIM1        0U   /* 最高：TIMG0 系统时基 1ms              */
#define IRQ_PRIO_TIM_DEFAULT 3U   /* 缺省定时器优先级                       */

/* ── USART 优先级（按实例独立设置）── */
#define IRQ_PRIO_USART4      1U   /* 高：摄像头数据通信                     */
#define IRQ_PRIO_USART2      2U   /* 中：步进电机 Emm42 通信                */
#define IRQ_PRIO_USART1      3U   /* 低：板载串口调试                       */
#define IRQ_PRIO_USART3      3U   /* 低：无线串口调试                       */

/* ── 编码器 EXTI（GPIO 外部中断，模拟正交编码器）── */
#define IRQ_PRIO_ENCODER_EXTI 1U   /* 高：编码器脉冲边沿捕获                 */

/* ── 其他 ── */
#define IRQ_PRIO_MPU6050     3U   /* 低：MPU6050 外部中断（未启用）          */
#define IRQ_PRIO_DEFAULT     3U   /* 最低：未指定中断的缺省值                */

/* ── sub-priority（M0+ 不支持，仅为 API 签名兼容保留）── */
#define IRQ_SUB_PRIO_MPU6050 0U

#endif /* __IRQ_PRIORITY_H */
