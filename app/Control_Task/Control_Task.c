#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "Tasks/Tasks.h"
#include "KEY.h"
#include "Encoder.h"
#include "G3507_Encoder.h"
#include "API_Motor.h"
#include "ICM42688.h"
#include "gray_adc.h"
#include "IR_Line.h"
#include "StepMotor.h"

/* ══════════════════════════════════════════════════════════════════════
 * 全局任务管理器实例（仅管理主循环执行的低频任务）
 * ══════════════════════════════════════════════════════════════════════ */
TaskManager tasks = {
    .buzzer_5ms = { .period = 5   },   /* 蜂鸣器/LED 调度    */
    .key_20ms   = { .period = 10  },   /* 按键轮询           */
    .print_50ms = { .period = 50  },   /* 串口：调试打印     */
    .oled_100ms = { .period = 100 },   /* 显示：OLED 刷新    */
};

/* ── 串口数据包由 My_Usart 模块 g_cam_data[] / g_cam_count 管理 ── */

/* ── 系统毫秒计数器（TIMG0 ISR 每 1ms +1，全局可读）── */
volatile uint32_t g_sys_tick_ms;

/*
 * Control_Task_TIM_Callback — TIMG0 1ms 时基中断回调
 */
void Control_Task_TIM_Callback(API_TIM_Id_t id)
{
    static uint8_t tick_5ms  = 0U;   /* 5ms 分频计数器  */
    static uint8_t tick_20ms = 0U;   /* 20ms 分频计数器 */

    if (id != API_TIM1)
    {
        return;
    }

    /* ── 0. 系统时基 @1ms ── */
    g_sys_tick_ms++;

    /* ── 1. 按键扫描 @1ms ── */
    Key_Tick();

    /* ── 2. 传感器数据解析刷新 - 5ms ── */
    tick_5ms++;
    if (tick_5ms >= 5U)
    {
        tick_5ms = 0U;
        GrayADC_Task(&g_graySensor); /* 感为无MCU灰度 */
        // IRLine_Task(&g_irLine);         /* 幻尔八路红外循迹 */

        ICM42688_ReadSensor();            /* ICM42688 6轴 burst读 + 偏航积分 */
        Direction_Control();              /* 灰度环PID计算输出 */

        usart_FrameTimeout_Check(&USART_DataTypeStruct, 100U); /* 帧超时 100ms 自动复位 */
    }

    /* ── 3. 控制 @20ms：编码器快照 + 任务调度 ── */
    tick_20ms++;
    if (tick_20ms >= 20U)
    {
        tick_20ms = 0U;

		G3507_Encoder_SnapshotAll();
		Encoder1_Speed =  -API_Encoder_GetFilteredSpeed(API_ENCODER_1);
		Encoder2_Speed =   API_Encoder_GetFilteredSpeed(API_ENCODER_2);
        // PID_Speed_Control(); /* 速度环 */
        // Direction_Test_Control(); /* 灰度环PID计算输出 */
        // YawTest_Control();  /* 偏航角环PID计算输出 */
        // LineFollow_Output(); /* 速度环 + 灰度环融合输出 */
        Task_Run();
	}

    /* ── 4. TaskManager：任务标志位（主循环消费）── */

    if (++tasks.buzzer_5ms.tick >= tasks.buzzer_5ms.period)
    {
        tasks.buzzer_5ms.tick = 0U;
        tasks.buzzer_5ms.flag = true;
    }

    if (++tasks.key_20ms.tick >= tasks.key_20ms.period)
    {
        tasks.key_20ms.tick = 0U;
        tasks.key_20ms.flag = true;
    }

    if (++tasks.print_50ms.tick >= tasks.print_50ms.period)
    {
        tasks.print_50ms.tick = 0U;
        tasks.print_50ms.flag = true;
    }

    if (++tasks.oled_100ms.tick >= tasks.oled_100ms.period)
    {
        tasks.oled_100ms.tick = 0U;
        tasks.oled_100ms.flag = true;
    }
}

/*
 * Control_Task_USART_Callback — USART 中断回调
 *
 * USART2 (JY61P/步进电机): 环形缓冲入队（StepMotor_RxPush），
 *   命令响应由阻塞函数消费。
 * USART4 (摄像头):     数据包直接在 ISR 内解析（纯整数状态机 <1µs/字节），
 *   完整帧立即捕获到 g_cam_data[] 全局数组，数据年龄最小。
 *
 * MSPM0 UART FIFO=4 字节，一次 ISR 可能读出多字节，必须循环排空。
 */
void Control_Task_USART_Callback(API_USART_Id_t id)
{
    uint32_t data;
    uint8_t rxValid;
    do
    {
        data    = 0U;
        rxValid = 0U;
        usart_irq_dispatch_by_id(id, &data, &rxValid);
        if (rxValid != 0U)
        {
            /* 步进电机应答 → 环形缓冲（阻塞命令从环缓取数） */
            if (id == API_USART2)
            {
                StepMotor_RxPush((uint8_t)data);
            }

            /* 摄像头数据包解析 */
            if (id == API_USART4)
            {
                usart_Dispose_Data(USART4, &USART_DataTypeStruct, (uint8_t)data);

                /* 收到完整数据包后立即捕获到全局变量 */
                if (USART_DataTypeStruct.state == 2U)
                {
                    USART_CamCapture();
                }
            }
        }
    } while (rxValid != 0U);
}

/* ══════════════════════════════════════════════════════════════════════
 * 非阻塞延时 — 基于 g_sys_tick_ms 的纯整数延时
 *
 * 原理：
 *   1ms ISR 递增 g_sys_tick_ms，Start() 记录当前 tick，
 *   IsDone() 比较已过 tick 数是否 ≥ 目标。
 *   不阻塞主循环，不关中断，支持 N 个独立实例。
 *
 * uint32_t 约 49 天溢出，无符号减法自动处理回绕，无 Bug。
 * ══════════════════════════════════════════════════════════════════════ */

/*
 * SysTick_GetMs — 返回系统上电以来的毫秒数。
 * 可在 ISR、主循环任意上下文调用。
 */
uint32_t SysTick_GetMs(void)
{
    return g_sys_tick_ms;
}

/*
 * NonBlockDelay_Start — 启动一个非阻塞延时。
 *
 * @param d  NonBlockDelay_t 实例指针
 * @param ms 延时目标（毫秒），>= 1
 *
 * 内部快照 g_sys_tick_ms 作为起始时刻，后续 IsDone() 自动计时。
 * 重复 Start() 会覆盖上一次未到期的延时。
 */
void NonBlockDelay_Start(NonBlockDelay_t *d, uint16_t ms)
{
    if ((d == NULL) || (ms == 0U))
    {
        return;
    }
    d->start_tick  = g_sys_tick_ms;
    d->duration_ms = ms;
}

/*
 * NonBlockDelay_IsDone — 检查延时是否到达。
 *
 * @retval 1  时间已到 / 从未 Start / ms==0
 * @retval 0  仍在计时中
 *
 * 可无限次反复调用，到达后每次返回 1。不会自动重置。
 * uint32_t 无符号减法天然处理 49 天溢出回绕。
 */
uint8_t NonBlockDelay_IsDone(NonBlockDelay_t *d)
{
    if (d == NULL)
    {
        return 1U;
    }
    if (d->duration_ms == 0U)
    {
        return 1U;   /* 从未启动过的延时，视为已到期 */
    }
    /* 无符号减法：tick 溢出回绕后差值仍然正确 */
    if ((g_sys_tick_ms - d->start_tick) >= d->duration_ms)
    {
        return 1U;
    }
    return 0U;
}
