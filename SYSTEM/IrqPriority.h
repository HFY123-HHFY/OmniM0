#ifndef __IRQ_PRIORITY_H
#define __IRQ_PRIORITY_H

/*
 * IrqPriority.h — 统一中断优先级管理
 *
 * 设计原则：
 * - 数字越小，优先级越高（NVIC 标准语义）
 * - 策略集中在这里，Core 层只接受优先级参数而不做决策
 * - 当前工程仅维护 MSPM0G3507
 *
 * 优先级分配：
 *  ┌──────────────────────────────────────────┐
 *  │  0  SysTick / 系统时钟                    │
 *  │  1  API_TIM 控制节拍 (1ms)               │ ← 所有 PID/控制回路的心脏
 *  │  2  MPU6050 EXTI                          │ ← 姿态数据，串级外环输入
 *  │  3  编码器 EXTI                           │ ← 速度内环反馈
 *  │  4  USART (×3)                            │ ← 通信丢包可重传
 *  │  5+ 缺省                                  │
 *  └──────────────────────────────────────────┘
 *
 * 注：
 * - PID 计算本身（pid_task_flag / Encoder_flag）运行在 main loop 中，
 *   不在 ISR 里执行，因此不占用 NVIC 优先级；但其数据来源 ISR 按上表分配。
 * - 姿态数据（MPU6050）是串级控制的外环输入，实时性要求高于编码器速度内环。
 *
 * MSPM0G3507: Cortex-M0+, __NVIC_PRIO_BITS=2, 范围 0~3
 */

/* G3507 仅有 2 bit 优先级 (0~3)，需要压缩映射 */
#if (ENROLL_MCU_TARGET == ENROLL_MCU_G3507)

#define IRQ_PRIO_TIM_CTRL    0U   /* 最高：1ms 控制节拍       */
#define IRQ_PRIO_MPU6050     1U   /* 高：姿态传感器（串级外环数据源） */
#define IRQ_PRIO_ENCODER     1U   /* 高：编码器（与 MPU6050 同级，G3507 仅 4 级） */
#define IRQ_PRIO_USART       2U   /* 中：串口收发              */
#define IRQ_PRIO_DEFAULT     3U   /* 低：未指定中断            */

/* G3507 EXTI 不使用子优先级，统一填 0 */
#define IRQ_SUB_PRIO_ENCODER 0U
#define IRQ_SUB_PRIO_MPU6050 0U
#else
#error "Unsupported ENROLL_MCU_TARGET for IrqPriority profile."
#endif

#endif /* __IRQ_PRIORITY_H */
