#ifndef __JY61P_H
#define __JY61P_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 数据更新标志位
 *===========================================================================*/
#define JY61P_ACC_UPDATE   0x01U
#define JY61P_GYRO_UPDATE  0x02U
#define JY61P_ANGLE_UPDATE 0x04U

/*===========================================================================
 * 传感器数据结构体
 *===========================================================================*/
typedef struct
{
    float acc_x;        /* 加速度 X（g）       */
    float acc_y;        /* 加速度 Y（g）       */
    float acc_z;        /* 加速度 Z（g）       */
    float gyro_x;       /* 角速度 X（°/s）     */
    float gyro_y;       /* 角速度 Y（°/s）     */
    float gyro_z;       /* 角速度 Z（°/s）     */
    float roll;         /* 滚转角（°）         */
    float pitch;        /* 俯仰角（°）         */
    float yaw;          /* 偏航角（°）         */
    float temp;         /* 温度（℃）           */
    uint8_t update_flags;
} JY61P_Data_t;

/*===========================================================================
 * API
 *===========================================================================*/

void JY61P_Init(void);

/*
 * 设置需要解析的数据包类型：
 *   0x51 — 仅加速度   0x52 — 仅角速度
 *   0x53 — 仅角度     0x00 — 全部
 *
 * 默认值：0x53（仅角度）。在 JY61P_Init 之后、主循环之前调用即可。
 */
void JY61P_SetActiveType(uint8_t packetType);

/*
 * JY61P_RxPush — 推入 1 字节到环形缓冲（中断上半部）。
 *
 * 由 USART 中断回调调用。只入队，不做任何解析。
 */
void JY61P_RxPush(uint8_t data);

/*
 * JY61P_Task — 处理环形缓冲中所有字节（中断下半部）。
 *
 * 在主循环 while(1) 中调用。逐字节取出 → 校验 → 浮点解析。
 */
void JY61P_Task(void);

/*
 * 获取传感器数据指针（只读）。
 */
const JY61P_Data_t *JY61P_GetData(void);

/*
 * 读取更新标志并自动清零。
 */
uint8_t JY61P_GetUpdateFlags(void);

/*
 * 读取加速度（单位 g）。任一参数可为 NULL 跳过。
 */
void JY61P_GetAcc(float *ax, float *ay, float *az);

/*
 * 读取角速度（单位 °/s）。任一参数可为 NULL 跳过。
 */
void JY61P_GetGyro(float *gx, float *gy, float *gz);

/*
 * 读取欧拉角（单位 °）。任一参数可为 NULL 跳过。
 */
void JY61P_GetAngle(float *roll, float *pitch, float *yaw);

/*
 * 读取温度（单位 ℃）。
 */
float JY61P_GetTemp(void);

#ifdef __cplusplus
}
#endif

#endif /* __JY61P_H */
