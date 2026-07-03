#ifndef __API_TIM_H
#define __API_TIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "G3507_tim.h"

typedef enum
{
	API_TIM1 = 0U,
	API_TIM2 = 1U,
	API_TIM3 = 2U,
	API_TIM4 = 3U,
	API_TIM5 = 4U
} API_TIM_Id_t;

#define API_TIM_CORE_TIMG0   (0U)
#define API_TIM_CORE_TIMG6   (1U)
#define API_TIM_CORE_TIMA0   (2U)
#define API_TIM_CORE_TIMA1   (3U)
#define API_TIM_CORE_TIMG7   (4U)
#define API_TIM_CORE_TIMG8   (5U)
#define API_TIM_CORE_TIMG12  (6U)

typedef struct
{
	API_TIM_Id_t id;
	uint8_t coreId;
} API_TIM_Config_t;

typedef void (*API_TIM_IrqHandler_t)(API_TIM_Id_t id);

/*
 * 定时器初始化接口：
 * id 选择定时器实例，periodMs 指定中断周期（毫秒）。
 */
void API_TIM_Register(const API_TIM_Config_t *configTable, uint8_t count);
void API_TIM_RegisterIrqHandler(API_TIM_Id_t id, API_TIM_IrqHandler_t handler);
void API_TIM_Init(API_TIM_Id_t id, uint32_t periodMs);
void API_TIM_HandleIrqByCoreId(uint8_t coreId);

#ifdef __cplusplus
}
#endif

#endif /* __API_TIM_H */
