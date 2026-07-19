#include "Buzzer.h"
#include "Control_Task/Control_Task.h"  /* NonBlockDelay_t */

/* ── 内部常量 ── */
#define BUZZER_CH_MAX  4U    /* Buzzer1 / LED1 / LED2 / LED3 */

/*
 * 每个通道独立状态，互不干扰。
 * busy=0 空闲，busy=1 正在计时等待关闭。
 */
typedef struct {
    NonBlockDelay_t delay;
    uint8_t         busy;
} Buzzer_Ch_t;

static Buzzer_Ch_t s_ch[BUZZER_CH_MAX];

/*
 * Buzzer_Light — 指定 LED/Buzzer 非阻塞亮指定 ms。
 *
 * @param id  LED 逻辑编号（LED1 / LED2 / LED3 / Buzzer1）
 * @param ms  高电平时长（毫秒）
 *
 * 立即打开，启动毫秒计时，函数立刻返回。重复调用同一通道覆盖上次延时。
 */
void Buzzer_Light(LED_Id_t id, uint32_t ms)
{
    uint8_t idx = (uint8_t)id;

    if (ms == 0U || idx >= BUZZER_CH_MAX) { return; }

    LED_Control(id, LED_HIGH);
    NonBlockDelay_Start(&s_ch[idx].delay, (uint16_t)ms);
    s_ch[idx].busy = 1U;
}

/*
 * Buzzer_Beep — 蜂鸣器非阻塞响 ms 毫秒。
 */
void Buzzer_Beep(uint32_t ms)
{
    Buzzer_Light(Buzzer1, ms);
}

/*
 * Buzzer_Alert — 蜂鸣器 + LED2 组合声光提示。
 * 所有任务函数可直接调用。
 */
void Buzzer_Alert(uint32_t ms)
{
    Buzzer_Light(Buzzer1, ms);
    Buzzer_Light(LED2, ms);
}

/*
 * Buzzer_Task — 主循环调度函数。
 *
 * 必须放在主循环 while(1) 中每次调用。
 * 扫描所有通道，超时则自动关闭对应的 LED/Buzzer。
 * 无活跃通道时仅 ~1µs。
 */
void Buzzer_Task(void)
{
    uint8_t i;

    for (i = 0U; i < BUZZER_CH_MAX; ++i)
    {
        if (s_ch[i].busy == 0U) { continue; }
        if (NonBlockDelay_IsDone(&s_ch[i].delay))
        {
            LED_Control((LED_Id_t)i, LED_LOW);
            s_ch[i].busy = 0U;
        }
    }
}
