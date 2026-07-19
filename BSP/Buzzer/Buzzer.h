#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>
#include "LED.h"     /* LED_Id_t / LED_Control / LED_HIGH / LED_LOW */

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════════════════
 * 非阻塞声光提示模块
 *
 * 基于 g_sys_tick_ms（1ms 全局 tick），替代阻塞式 LED_Turn / Delay_ms。
 *
 * 用法：
 *   Buzzer_Beep(200);               // 蜂鸣器响 200ms，非阻塞立刻返回
 *   Buzzer_Light(LED2, 300);        // LED2 亮 300ms
 *   Buzzer_Alert();                 // 蜂鸣器 + LED2 各 200ms（声光组合）
 *   Buzzer_Task();                  // 主循环每次调用，超时自动关闭
 *
 * 支持 4 个独立通道（Buzzer1 / LED1 / LED2 / LED3），互不干扰。
 * ══════════════════════════════════════════════════════════════════════ */

void Buzzer_Beep(uint32_t ms);                /* 蜂鸣器非阻塞响 ms 毫秒          */
void Buzzer_Light(LED_Id_t id, uint32_t ms);  /* LED 非阻塞亮 ms 毫秒            */
void Buzzer_Alert(uint32_t ms);               /* 蜂鸣器 + LED2 同步声光提示 ms 毫秒 */
void Buzzer_Task(void);                       /* 主循环调度（超时自动关）         */

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H */
