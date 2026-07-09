#ifndef __IRQ_PRIORITY_H
#define __IRQ_PRIORITY_H

/*
 * IrqPriority.h — 统一中断优先级管理
 *
 * MSPM0G3507: Cortex-M0+, __NVIC_PRIO_BITS=2, 范围 0~3（数字越小优先级越高）
 *
 * 优先级分配：
 *  ┌──────┬──────────────────────────────────────────────┐
 *  │  0   │ TIM1 — 控制节拍 1ms（方向环 5ms + 灰度采集）  │
 *  │  1   │ TIM2 — 编码器 20ms（速度环 + Control_Run）    │
 *  │  2   │ USART3 — JY61P 陀螺仪（实时数据流，需低延迟）  │
 *  │  3   │ USART1/2/4 + 缺省（调试串口/MPU6050等）       │
 *  └──────┴──────────────────────────────────────────────┘
 */

/* ── TIM 优先级 ── */
#define IRQ_PRIO_TIM1        0U   /* 最高：1ms 控制节拍         */
#define IRQ_PRIO_TIM2        1U   /* 高：编码器 + Control_Run  */
#define IRQ_PRIO_ENCODER     2U   /* 中：编码器 EXTI           */
#define IRQ_PRIO_TIM_DEFAULT 3U   /* 缺省                      */

/* ── USART 优先级（按实例独立设置） ── */
#define IRQ_PRIO_USART3      2U   /* 中：JY61P 陀螺仪      */
#define IRQ_PRIO_USART1      3U   /* 低：串口调试          */
#define IRQ_PRIO_USART2      3U   /* 低：预留摄像头协议数据包 */
#define IRQ_PRIO_USART4      3U   /* 低：未用                  */

/* ── 其他 ── */
#define IRQ_PRIO_MPU6050     3U   /* 低：MPU6050 姿态传感器     */
#define IRQ_PRIO_DEFAULT     3U   /* 最低：未指定中断           */

/* ── sub-priority（M0+ 不支持，仅为兼容保留） ── */
#define IRQ_SUB_PRIO_ENCODER 0U
#define IRQ_SUB_PRIO_MPU6050 0U

#endif /* __IRQ_PRIORITY_H */
