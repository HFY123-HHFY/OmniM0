#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "KEY.h"
#include "Encoder.h"
#include "G3507_Encoder.h"
#include "gray_adc.h"
#include "jy61p.h"

/* ══════════════════════════════════════════════════════════════════════
 * 全局任务管理器实例（仅管理主循环执行的低频任务）
 * ══════════════════════════════════════════════════════════════════════ */
TaskManager tasks = {
    .print_50ms = { .period = 50  },   /* 串口：调试打印    */
    .oled_100ms = { .period = 100 },   /* 显示：OLED 刷新   */
};

/* ── 串口数据包缓存（USART 中断回调使用）── */
int16_t USART_Packet_Data[USART_PACKET_DATA_LEN] = {0};
uint8_t USART_Packet_Count = 0;

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

    /* ── 1. 按键扫描 @1ms ── */
    Key_Tick();

    /* ── 2. 方向环 @5ms：灰度采集 + 方向 PID ── */
    tick_5ms++;
    if (tick_5ms >= 5U)
    {
        tick_5ms = 0U;
        GrayADC_Task(&g_graySensor);
        if (!Control_IsTurning())
        {
            Direction_Control();
        }
    }

    /* ── 3. 速度环 @20ms：编码器快照 + 循线控制 ── */
    tick_20ms++;
    if (tick_20ms >= 20U)
    {
        tick_20ms = 0U;

        G3507_Encoder_SnapshotAll();
        Encoder1_Speed = API_Encoder_GetSpeed(API_ENCODER_1);
        Encoder2_Speed = API_Encoder_GetSpeed(API_ENCODER_2);
        Control_Run((float)Encoder1_Speed, (float)Encoder2_Speed);
    }

    /* ── 4. TaskManager：低频任务标志位（主循环消费）── */

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
 * JY61P RxPush 只做环形缓冲入队（< 1µs），数据包解析由主循环 JY61P_Task() 完成。
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
            if (id == API_USART3)
            {
                JY61P_RxPush((uint8_t)data);  /* 只入队，解析交给主循环 JY61P_Task() */
            }
        }
    } while (rxValid != 0U);
}
