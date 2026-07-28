#ifndef __API_ENCODER_H
#define __API_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * API Encoder 层职责：
 * 1) 提供统一的编码器接口（读速度）；
 * 2) 调用 G3507 Core 层实现。
 */
#include "G3507_Encoder.h"

/* 逻辑编码器 ID */
typedef enum
{
	API_ENCODER_1 = 0U,
	API_ENCODER_2 = 1U
} API_Encoder_Id_t;

/* 编码器速度全局变量（EMA 滤波后，供 PID 速度环使用） */
extern int16_t Encoder1_Speed;
extern int16_t Encoder2_Speed;

/* ══════════════════════════════════════════════════════════════════════
 * EMA 低通滤波（消除编码器脉冲抖动 / 齿槽转矩噪声）
 *
 * 公式：filtered += (raw - filtered) * alpha / 65536
 * alpha 越小 → 滤波越强，响应越慢；alpha 越大 → 越接近原始值
 *
 * 推荐值（20ms 采样周期）：
 *   - 0.3 (19661)  ← 默认，适合大多数电机，~100ms 阶跃响应
 *   - 0.5 (32768)  ← 轻滤波，噪声敏感场景慎用
 *   - 0.7 (45875)  ← 极轻，接近原始值
 *   - 0      (0)   ← 无滤波，等价于 GetSpeed
 * ══════════════════════════════════════════════════════════════════════ */

#define ENCODER_FILTER_ALPHA_Q16  19661U   /* ≈0.3，默认滤波强度 */

/* 配置编码器 EMA 滤波系数（Q0.16，0-65535）。
 * 应在 Init 之后、周期读取之前调用一次。 */
void API_Encoder_SetFilterAlpha(API_Encoder_Id_t id, uint16_t alpha_q16);

/* 读取滤波后的编码器速度。
 * 每次调用内部执行一次 EMA 更新（基于当前 stable 快照值），
 * 调用频率应与 SnapshotAll 一致（20ms），否则滤波时间常数会偏移。 */
int16_t API_Encoder_GetFilteredSpeed(API_Encoder_Id_t id);

/*
 * 定时器输入捕获通道常量
 * 用于 hw_config 中指定编码器信号映射到哪个 TIM 通道。
 */
#define API_ENCODER_CH1  (1U)
#define API_ENCODER_CH2  (2U)

/*
 * Core 编码器 ID：
 * G3507: 用编码器序号（0/1）区分
 */

#define API_ENCODER_CORE_ENC0  (0U)
#define API_ENCODER_CORE_ENC1  (1U)

/*
 * 编码器配置表项：
 * - id:     逻辑编码器 ID
 * - coreId: Core 层实例 ID（TIM 编号或编码器序号）
 * - chA:    A 相信号对应的定时器输入捕获通道（CH1/CH2）
 * - portA/pinA: A 相 GPIO
 * - chB:    B 相信号对应的定时器输入捕获通道（CH1/CH2）
 * - portB/pinB: B 相 GPIO
 */
typedef struct
{
	API_Encoder_Id_t id;
	uint8_t          coreId;
	uint8_t          chA;
	void            *portA;
	uint32_t         pinA;
	uint8_t          chB;
	void            *portB;
	uint32_t         pinB;
} API_Encoder_Config_t;

/*
 * 编码器注册：登记板级编码器资源表（不初始化硬件）。
 */
void API_Encoder_Register(const API_Encoder_Config_t *configTable, uint8_t count);

/*
 * 编码器初始化：根据注册表启动指定编码器硬件。
 */
void API_Encoder_Init(API_Encoder_Id_t id);

/*
 * 读取编码器速度：返回最近一次 SnapshotAll 快照的计数值（带方向）。
 * 计数器清零由 SnapshotAll 在固定周期 ISR（20ms）中完成，
 * 调用方应在 SnapshotAll 之后立即读取，以获得恒定采样窗口的速度值。
 */
int16_t API_Encoder_GetSpeed(API_Encoder_Id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __API_ENCODER_H */
