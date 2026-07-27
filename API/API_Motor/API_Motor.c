#include "API_Motor.h"

#if (MOTOR_DRIVER == MOTOR_DRIVER_AT4950)
#include "AT4950.h"
#elif (MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
#include "TB6612.h"
#endif

/*
 * 编译器在 -O2 下会将下面两个函数体内联：
 *   API_Motor_Init()    → 直接展开为 AT4950_Init() 或 TB6612_Init()
 *   API_Motor_SetSpeed() → 直接展开为对应底层调用
 * 零额外开销，和直接调用无区别。
 */

void API_Motor_Init(void)
{
#if (MOTOR_DRIVER == MOTOR_DRIVER_AT4950)
	AT4950_Init();
#elif (MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
	TB6612_Init();
#endif
}

void API_Motor_SetSpeed(int16_t speedA, int16_t speedB)
{
#if (MOTOR_DRIVER == MOTOR_DRIVER_AT4950)
	AT4950_SetSpeed(speedA, speedB);
#elif (MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
	TB6612_SetSpeed(speedA, speedB);
#endif
}
