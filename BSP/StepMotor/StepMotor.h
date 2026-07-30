#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Emm42 V5.0 步进闭环电机驱动
 *
 * 硬件：UART 通信（API_USART2, 115200 bps），命令-应答协议
 * 校验字节：固定 0x6B
 *
 * 首次校准（仅一次，电机必须空载！）：
 *   // Stepmotor_CalibrateOnce();   // 取消注释 → 烧录 → 校准完成 → 注释 → 烧录
 *
 * 每次上电：
 *   Stepmotor_BootInit();  // 等驱动板上线 → 使能 → 自动回零
 *   Stepmotor_ConfigMove(250.0f, 200.0f);
 *
 * PID 控制（20ms 周期，非阻塞）：
 *   Stepmotor_SetAngle(STEPMOTOR1, target);   // 发完立即返回
 *===========================================================================*/

/* ── 电机 ID ── */
#define STEPMOTOR1  0x01U
#define STEPMOTOR2  0x02U

/* ── 运动模式 ── */
#define MOTOR_MODE_REL  0x00U   /* 相对当前位置 */
#define MOTOR_MODE_ABS  0x01U   /* 绝对位置       */

/* ── 电机状态标志位（0x3A 应答字节）── */
#define MOTOR_STAT_ENABLED    0x01U   /* 已使能           */
#define MOTOR_STAT_IN_POS     0x02U   /* 已到位           */
#define MOTOR_STAT_STALL_NOW  0x04U   /* 正在堵转         */
#define MOTOR_STAT_STALL_LOCK 0x08U   /* 堵转锁死（需复位）*/

/* ── 错误码 ── */
typedef enum {
    MOTOR_OK            =  0,
    MOTOR_ERR_NONE      = -1,   /* 驱动板无应答（检查接线/供电/波特率） */
    MOTOR_ERR_TIMEOUT   = -2,   /* 操作超时                           */
    MOTOR_ERR_STALL     = -3,   /* 堵转                               */
    MOTOR_ERR_HOME_FAIL = -4    /* 回零失败（可能没保存过零点）        */
} MotorErrCode;

/*===========================================================================
 * ① 封装好的初始化流程（main 中直接调用，无需关心内部细节）
 *===========================================================================*/

/* 模块初始化：清环形缓冲，每次上电在 API_USART_Init 之后调用。 */
void Stepmotor_Init(void);

/*
 * 首次编码器校准 + 设零点 — 仅需执行一次！
 *
 * ⚠️ 校准前电机必须空载！校准完成后 GoHome 验证 → 死循环。
 *    用户重新注释此调用并烧录正常固件。
 *
 * 调用方式（main.c 中，校准一次后立即注释掉）：
 *   // Stepmotor_CalibrateOnce();   // ← 仅首次！
 */
void Stepmotor_CalibrateOnce(void);

/*
 * 每次上电的电机初始化：等驱动板上线 → 使能 → 自动回零。
 *
 * @retval MOTOR_OK       成功，电机已在零点，可进入 PID 控制
 * @retval MOTOR_ERR_NONE 驱动板 5s 内未上线（检查接线/供电/波特率）
 */
MotorErrCode Stepmotor_BootInit(void);

/*===========================================================================
 * ② 通信检测与使能
 *===========================================================================*/

MotorErrCode Stepmotor_SelfTest(uint8_t id);      /* 检测驱动板通信（~200ms）    */
MotorErrCode Stepmotor_Enable(uint8_t id);        /* 使能电机（~500ms）           */
MotorErrCode Stepmotor_Disable(uint8_t id);       /* 失能电机（~500ms）           */
MotorErrCode Stepmotor_Stop(uint8_t id);          /* 急停（~500ms）               */

/*===========================================================================
 * ③ 首次校准（仅一次，电机必须空载！）
 *===========================================================================*/

MotorErrCode Stepmotor_Calibrate(uint8_t id, uint32_t timeoutMs);   /* 编码器校准 */
MotorErrCode Stepmotor_SetOrigin(uint8_t id, uint8_t saveToFlash);  /* 保存回零原点 */
MotorErrCode Stepmotor_GoHome(uint8_t id, uint32_t timeoutMs);      /* 回零并阻塞等待 */
MotorErrCode Stepmotor_ResetStall(uint8_t id);                      /* 清除堵转锁死 */

/*===========================================================================
 * ④ 运动控制
 *===========================================================================*/

/*
 * 配置运动参数（速度/加速度），后续 SetAngle / MoveTo 沿用。
 * 调用一次即可，后续可随时改。
 */
void Stepmotor_ConfigMove(float speed_rpm, float accel_rpm_s);

/*
 * ★ 非阻塞绝对角度 — PID 输出执行函数。
 * 速度/加速度沿用 ConfigMove 配置，发完立即返回。
 */
void Stepmotor_SetAngle(uint8_t id, float angle_deg);

/*
 * 非阻塞相对角度 — 在当前位置基础上转动 angle_deg 度。
 */
void Stepmotor_MoveBy(uint8_t id, float angle_deg);

/*
 * 非阻塞位置指令（完整参数）。
 *
 * @param rpm   转速（RPM），0 = 沿用 ConfigMove 设置
 * @param accel 加速度档位，0 = 沿用 ConfigMove 设置
 */
void Stepmotor_MoveTo(uint8_t id, float angle, uint16_t rpm,
                      uint8_t accel, uint8_t mode);

/*
 * 阻塞位置指令（等到位或超时才返回）。
 */
MotorErrCode Stepmotor_GoTo(uint8_t id, float angle, uint16_t rpm,
                            uint8_t accel, uint8_t mode, uint32_t timeoutMs);

/*===========================================================================
 * ⑤ 状态查询（均阻塞 ~200ms）
 *===========================================================================*/

float     Stepmotor_ReadAngle(uint8_t id);    /* 实时角度（°）   */
float     Stepmotor_ReadSpeed(uint8_t id);    /* 实时转速（RPM） */
uint8_t   Stepmotor_ReadStatus(uint8_t id);   /* 状态标志位      */

/*===========================================================================
 * ⑥ ISR 接口（USART 中断回调中使用）
 *
 * Control_Task_USART_Callback 中调用：
 *   if (id == API_USART2) { StepMotor_RxPush((uint8_t)data); }
 *
 * 阻塞命令从环形缓冲轮询取数，非阻塞命令（SetAngle/MoveBy/Stop）不接收应答。
 *===========================================================================*/

void StepMotor_RxPush(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __STEPMOTOR_H */
