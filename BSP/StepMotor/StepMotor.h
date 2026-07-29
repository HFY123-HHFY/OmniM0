#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 张大头 ZDT_X系列 步进闭环电机驱动
 *
 * 硬件：UART 通信（API_USART3, 115200 bps），命令-应答协议
 * 校验和：所有字节累加取低 8 位
 *
 * 典型初始化流程（main.c 中一次）：
 *   StepMotor_Init(0x01);     // 地址 = 0x01
 *   StepMotor_Enable();       // 使能（阻塞等待应答 ~20ms）
 *   StepMotor_ConfigMove(600, 400);  // 配置转速 600RPM / 加速度 400RPM/s
 *
 * PID 控制流程（20ms 周期）：
 *   float target = pid_output;              // PID 输出即目标角度
 *   StepMotor_SetAngle(target);             // 发送指令，立即返回（非阻塞）
 *===========================================================================*/

/* ── 电机状态标志位 ── */
#define STEPMOTOR_STAT_ENABLED   0x01U
#define STEPMOTOR_STAT_INPOS     0x02U
#define STEPMOTOR_STAT_STALL     0x04U
#define STEPMOTOR_STAT_PROTECT   0x08U

/*===========================================================================
 * ① 初始化与使能（启动阶段调用一次）
 *===========================================================================*/

void    StepMotor_Init(uint8_t addr);        /* 初始化，addr=电机ID(默认0x01) */
int8_t  StepMotor_Enable(void);              /* 使能电机（阻塞等待应答）       */
int8_t  StepMotor_Disable(void);             /* 失能电机                       */

/*===========================================================================
 * ② PID 输出接口（周期调用，非阻塞）
 *
 * 这是摆杆控制的核心函数。先 ConfigMove 配置一次速度/加速度，
 * 然后 PID 每周期把输出角度传给 SetAngle 即可。
 *===========================================================================*/

/*
 * 配置运动参数，后续 SetAngle 沿用。
 *
 * @param speed  最大转速（RPM），典型值 300~800
 * @param accel  加减速加速度（RPM/s），典型值 200~500
 *
 * 调用一次即可，后续可随时改。
 */
void StepMotor_ConfigMove(float speed, float accel);

/*
 * ★ 设置电机绝对角度 — PID 输出执行函数（非阻塞）。
 *
 * 发送梯形曲线位置指令后立即返回，不等待应答。
 * 电机内部自行完成平滑加减速，新指令自动覆盖旧指令。
 *
 * @param angle_deg  目标角度（°），正/负表示 CW/CCW
 *
 * 用法：
 *   // 初始化时：
 *   StepMotor_ConfigMove(600.0f, 400.0f);
 *
 *   // PID 控制循环中（20ms 周期）：
 *   float correction = pid_output;   // 位置环 PID 输出
 *   StepMotor_SetAngle(correction);  // 摆杆立刻跟踪
 */
void StepMotor_SetAngle(float angle_deg);

/*
 * 设置电机相对角度（非阻塞）。
 *
 * 在当前位置基础上再转动 angle_deg 度。
 * 速度/加速度沿用 ConfigMove 的配置。
 */
void StepMotor_MoveBy(float angle_deg);

/*===========================================================================
 * ③ 查询（读取当前状态）
 *===========================================================================*/

float     StepMotor_GetAngle(void);          /* 实时角度（°）        */
float     StepMotor_GetSpeed(void);          /* 实时转速（RPM）      */
uint8_t   StepMotor_GetStatus(void);         /* 状态标志位（位域）    */
uint8_t   StepMotor_IsInPosition(void);      /* 到位？1=是 0=否      */
uint8_t   StepMotor_IsStalled(void);         /* 堵转？1=是 0=否      */

/*===========================================================================
 * ④ 辅助控制
 *===========================================================================*/

void      StepMotor_Stop(void);              /* 急停（非阻塞，所有模式通用）  */
int8_t    StepMotor_ClearAngle(void);        /* 当前位置清零（设为零点）      */
int8_t    StepMotor_ReleaseStall(void);      /* 解除堵转保护                  */

/*===========================================================================
 * ⑤ ISR 接口（中断回调专用）
 *===========================================================================*/

/*
 * 推入 1 字节到接收缓冲区（ISR 上半部，极快）。
 *
 * 已在 Control_Task.c 中自动路由：USART3 中断 → StepMotor_RxPush。
 * 用户代码不需要手动调用。
 */
void StepMotor_RxPush(uint8_t data);

/*
 * 处理应答帧（主循环调用，解析环形缓冲区中的响应）。
 *
 * 对于非阻塞命令（SetAngle/MoveBy/Stop），由本函数异步更新内部状态。
 * 对于查询命令（GetAngle/GetStatus），由对应的 getter 函数内部同步调用。
 */
void StepMotor_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEPMOTOR_H */
