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
 * ⑤ 回零操作
 *
 * 回零流程（两步）：
 *   1. 校准阶段（仅一次）：手动把电机轴转到想要的零点位置 →
 *      StepMotor_SetZero(1) → 零点存入驱动器 Flash，永久保存。
 *   2. 每次上电：StepMotor_Enable() → StepMotor_GoHome(5000) →
 *      电机自动转回零点 → 进入正常控制。
 *===========================================================================*/

/*
 * 设置单圈回零零点（仅需调用一次）。
 *
 * 将电机当前的物理位置保存为绝对零点。参数存入驱动器 Flash，
 * 掉电不丢失。之后再调用 GoHome 就会回到这个位置。
 *
 * @param saveToFlash  1=存入 Flash（推荐），0=仅本次上电有效
 * @retval  0  设置成功
 * @retval -1  超时 / 应答错误
 *
 * 调用时机：机械结构安装完成后，手动把摆杆拨到你想要的角度，
 *          然后调用此函数。之后不要再调（除非重新校准机械）。
 */
int8_t StepMotor_SetZero(uint8_t saveToFlash);

/*
 * 触发单圈回零（每次上电后调用）。
 *
 * 发送回零指令后，阻塞等待电机到达零点（轮询到位标志）。
 * 回零方向和模式由驱动器内部菜单（O_Mode / O_Dir）决定。
 *
 * @param timeoutMs  最大等待时间（ms），默认 5000（5 秒）
 * @retval  0  回零成功（已到达零点）
 * @retval -1  超时 / 堵转 / 回零失败
 *
 * 调用时机：StepMotor_Enable() 之后、进入控制循环之前。
 *          阻塞等待回零完成，确保每次上电后摆杆从固定零点出发。
 */
int8_t StepMotor_GoHome(uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* __STEPMOTOR_H */
