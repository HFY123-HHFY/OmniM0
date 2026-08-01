# OmniLayer 工程架构深度解析（G3507 单平台）

> 本文档用于快速恢复对 OmniLayer 当前架构的完整认知。
> 当前仓库已收敛为单 MCU 目标：TI MSPM0G3507。

---

## 1. 项目元信息

| 项目 | 详情 |
|------|------|
| 名称 | OmniM0 |
| 定位 | 面向电赛场景的 G3507 分层开发框架（循线机器人） |
| 构建工具 | CMake + GCC ARM Embedded + OpenOCD |
| IDE 兼容 | VS Code（主）+ Keil MDK（兼容保留） |
| 主维护方向 | 裸机主线 + 后续 RTOS 演进 |
| 作者 | Hu Fangyuan |
| 联系方式 | 634591772@qq.com |

---

## 2. 目标平台

| MCU | 架构 | 内核 | 主频 | NVIC 优先级位 |
|-----|------|------|------|--------------|
| TI MSPM0G3507 | ARM | Cortex-M0+ | **80MHz**（SYS 层超频） | 2 bit（0-3，仅 4 级） |

---

## 3. 分层架构总览

```text
A_Entry/                程序入口（main.c）
app/                    应用层（控制逻辑、任务调度、算法）
BSP/                    板级设备层（LED/KEY/OLED/ICM42688/A4950/JY61P/GrayADC 等）
Enroll/                 注册层（板级资源映射与注册）
API/                    接口层（片内外设抽象 + I2C/SPI 协议层）
Core/MSPM0G3507/        核心层（G3507 底层实现）
SYSTEM/                 系统层（系统初始化、总线配置、中断策略）
Drivers/Drivers_M0G3507 驱动资源层（启动文件/CMSIS/DriverLib）
```

调用关系：
- app 调用 BSP/API。
- BSP 通过 API 访问底层外设。
- Enroll 在启动阶段完成资源注册，把逻辑 ID 绑定到物理资源。
- API 统一分发到 Core/MSPM0G3507。

---

## 4. 核心设计模式

### 4.1 注册层模式（Enroll）

本质：用编译期映射表把逻辑资源 ID 映射到具体 GPIO、定时器、串口实例。

```text
G3507_hw_config.h (映射宏)
  -> Enroll.c (X-Macro 展开配置表)
  -> Enroll_xxx_Register()
  -> API/BSP Register()
  -> 运行期按逻辑 ID 使用资源
```

### 4.2 两阶段初始化（Register -> Init）

- 阶段 1：Enroll_xxx_Register() 登记配置表。
- 阶段 2：API_xxx_Init() 激活硬件。

### 4.3 软件总线双分层（I2C/SPI）+ ISR 抢占保护

- API 协议层：负责起始、停止、收发字节等协议流程。
- Core 底层层：负责 GPIO 翻转和延时。
- **SPI 上下文保护**：`soft_spi_hal.h` 提供 `soft_spi_context_t` + `save/restore`，
  ISR 中的 SPI 事务（如 ICM42688）可安全抢占主循环的 SPI 事务（如 OLED），
  两路 SPI 使用不同 GPIO 引脚 + 全局状态变量快照/恢复，互不干扰。
  - 模式：ISR → `save` → 切 SPI2 → 读传感器 → `restore` → 主循环 SPI1 无缝继续
  - OLED（SPI1）与 ICM42688（SPI2）共用软件 SPI 层但物理引脚隔离

### 4.4 单一时基前后台架构（TIMG0 1ms）

**仅使用一个定时器 TIMG0** 作为系统时基（1ms 中断）。

ISR 中直接执行时序敏感的控制任务：
- g_sys_tick_ms++ + Key_Tick @1ms（按键消抖，内部 10 分频，30ms 消抖窗口）
- ICM42688_ReadSensor + GrayADC_Task + 出入线检测 + Direction_Control @5ms
- Encoder Snapshot + GetFilteredSpeed + 控制输出 @20ms
- TaskManager tick 计数（只置标志位，供主循环消费）

**ICM42688_ReadSensor 在 ISR 5ms 中读取（数据年龄 ≤5ms）。**
**方向 PID 使用 Q16.16 整数。ICM42688 含 float 转换（atan2f/sqrtf），5ms 槽合计最坏 ~185µs（3.7% CPU）。**

主循环仅负责非实时任务：
- key_Get（按键事件同步，20ms）
- OLED 刷新（100ms，由 TaskManager 标志位驱动）
- 蜂鸣器/LED 调度（5ms）
- printf 打印（50ms，由 TaskManager 标志位驱动）

### 4.5 统一中断优先级策略

统一策略放在 `SYSTEM/IrqPriority.h`。M0+ 仅 4 级（0~3）：

| 优先级 | 外设 | 说明 |
|:---:|------|------|
| 0 | TIMG0 | 系统时基 1ms（ISR 调度核心，不可抢占） |
| 1 | Encoder EXTI / USART4 | 编码器脉冲边沿捕获 / 摄像头数据通信 |
| 2 | USART2 | 步进电机 Emm42 通信 |
| 3 | USART1/3 + MPU6050 + 缺省 | 调试串口 / 未启用外设 |

编码器 EXTI 优先级统一在 `SYSTEM/IrqPriority.h` 中管理（`IRQ_PRIO_ENCODER_EXTI 1U`）。

### 4.6 ICM42688 数据流架构（当前主力 IMU）

ICM42688 为当前主力 IMU（JY61P 保留备用）。数据按需通过 `ICM42688_GetSnapshot()` 读取：

```text
┌─ 主循环 / Task_Run（20ms ISR 插槽）─────────────────┐
│  ICM42688_GetSnapshot(&snap) → snap.yaw / roll / pitch│
│    → 双缓冲原子读，零 SPI 开销                         │
│    → 数据由 ICM42688 后台 SPI 事务维护                 │
└──────────────────────────────────────────────────────┘
                    ↓
  YawPid_Calc(snap.yaw) → 整数 PID（Q16.16）
  或 Drive_YawSpeed() → 速度环 + 偏航角环融合输出
```

设计原则：
- ICM42688 通过 SPI 上下文保护（`soft_spi_hal_save/restore`）安全访问，
  即使主循环正在做 OLED SPI 刷新也不会冲突。
- 控制代码通过 `ICM42688_GetSnapshot()` 原子快照读取数据，零额外 SPI。
- 偏航积分基于 `g_sys_tick_ms` 自动计算 dt，首次自动归零。

### 4.7 非阻塞延时

基于 TIMG0 1ms 全局 tick（`g_sys_tick_ms`）的纯整数延时系统：

```c
NonBlockDelay_t d;
NonBlockDelay_Start(&d, 200);        // 启动 200ms 延时，立刻返回
if (NonBlockDelay_IsDone(&d)) { ... } // 轮询是否到期

Buzzer_Light(Buzzer1, 200);      // 非阻塞蜂鸣器（替代阻塞版 LED_Turn）
Buzzer_Task();                   // 主循环调用，超时自动关
```

支持 N 个独立通道同时计时（蜂鸣器 + 多路 LED），`uint32_t` 无符号减法天然处理 49 天溢出回绕。

### 4.8 MSPM0 UART FIFO 与中断触发

MSPM0G3507 的 UART 有 **4 字节硬件 RX FIFO**，默认中断阈值 = 半满（2 字节）。

**重要**：必须把阈值设为 1 字节，否则帧尾字节会卡在 FIFO 里，直到下一帧到达才释放。

```c
// G3507_usart.c — 所有 UART 实例初始化时统一设置
DL_UART_Main_setRXFIFOThreshold(map.regs,
                                DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
```

### 4.9 非阻塞蜂鸣器

替代阻塞式 `LED_Turn()`，基于 `NonBlockDelay_t`：

```c
Buzzer_Light(Buzzer1, 200);  // 响 200ms，立刻返回
Buzzer_Task();               // 主循环调用，超时自动关

// 多通道独立计时
Buzzer_Light(Buzzer1, 200);
Buzzer_Light(LED1, 500);
```

### 4.10 编码器 EMA 低通滤波 + 堵转归零

编码器速度值经过 EMA 低通滤波后再送入 PID 速度环：

```text
EXTI ISR → s_encoderRaw[±1]
              ↓
TIMG0 20ms → SnapshotAll() → raw→stable, raw=0
              ↓
         API_Encoder_GetFilteredSpeed()
              ↓
         EMA: filtered += (raw - filtered) * α / 65536   (Q16.16 定点)
              ↓
         堵转检测: 连续 2 次 raw=0 → 滤波值立即归零（消除 EMA 长尾残留）
              ↓
         Encoder1_Speed / Encoder2_Speed → 速度环 PID
```

- 默认 α=0.3（19661 Q16），20ms 采样，阶跃响应 ~100ms
- `API_Encoder_SetFilterAlpha(id, alpha_q16)` 可在线调整滤波强度
- `API_Encoder_GetSpeed()` 仍返回原始快照值（绕过滤波）

---

## 5. API 层外设接口（当前）

| API 头文件 | 功能 | G3507 状态 |
|-----------|------|------------|
| API/inc/gpio.h | GPIO 输入输出 | 已实现 |
| API/inc/usart.h | 串口通信 | 已实现（4 路，独立优先级） |
| API/inc/pwm.h | PWM 输出 | 已实现（TIMA1 + TIMG8 双定时器支持） |
| API/inc/tim.h | 定时器中断 | 已实现（独立优先级） |
| API/inc/adc.h | ADC 采集 | 已实现 |
| API/inc/exti.h | 外部中断 | 已实现 |
| API/inc/Encoder.h | 编码器接口 + EMA 滤波 | 已实现（EXTI 软件编码器） |
| API/API_I2C/API_I2C.h | 软件 I2C 协议 | 已实现 |
| API/API_SPI/API_SPI.h | 软件 SPI 协议 + 上下文保护 | 已实现 |

---

### 5.1 PWM 多定时器支持（5 路）

`G3507_pwm.c` 统一支持 **5 种定时器**（TIMG7/TIMG6/TIMA0/TIMA1/TIMG8），8 个 PWM 通道：

| API | 通道 | 引脚 | 硬件定时器 | CCP | 电源域 | 时钟 | Period | 频率 | 用途 |
|-----|------|------|-----------|-----|--------|------|--------|------|------|
| TIM1 | CH1 | PB15 | TIMG7 | CCP0 | PD1 | 80M | 4000 | 20k | A4950 AIN1 |
| TIM2 | CH1 | PB7 | TIMG6 | CCP1 | PD1 | 80M | 4000 | 20k | A4950 AIN2 |
| TIM3 | CH1 | PB13 | TIMA0 | CCP3 | PD1 | 80M | 4000 | 20k | A4950 BIN1 |
| TIM3 | CH2 | PB9 | TIMA0 | CCP1 | PD1 | 80M | 4000 | 20k | A4950 BIN2 |
| TIM4 | CH1 | PA28 | TIMA1 | CCP0 | PD1 | 80M | 4000 | 20k | 预留 |
| TIM4 | CH2 | PA31 | TIMA1 | CCP1 | PD1 | 80M | 4000 | 20k | 预留 |
| TIM5 | CH1 | PA29 | TIMG8 | CCP0 | PD0 | 40M | 2000 | 20k | 预留 |
| TIM5 | CH2 | PA30 | TIMG8 | CCP1 | PD0 | 40M | 2000 | 20k | 预留 |

- `API_PWM_CORE_CCP0~3`：4 个比较通道宏，TIMA0 使用 CCP3（PB13）
- `G3507_PWM_IsTimerG`：自动识别 TIMG 类（TIMG6/7/8）vs TIMA 类（TIMA0/1），分支到对应 API
- `HW_PWM_MAP` 中改 `coreTimId` + `coreChannel` 即可重新分配引脚，底层全自动适配
- 输出极性：LACT LOW / CDACT HIGH（边缘对齐减计数）→ `duty=0` 恒低，`duty=period` 恒高（刹车用）
- **PWM Period = 4000，满占空比 = 4000，对应 `API_MOTOR_MAX_DUTY`**

---

## 6. BSP 层器件

| 模块 | 路径 | 接口类型 | 状态 |
|------|------|----------|------|
| LED | BSP/LED/ | GPIO | 已实现 |
| KEY | BSP/KEY/ | GPIO（消抖，4 键协议） | 已实现 |
| OLED | BSP/OLED/ | SPI（软，SPI1） | 已实现 |
| **StepMotor** | BSP/StepMotor/ | UART（USART2, 115200） | **已实现（Emm42 V5.0 闭环驱动）** |
| **ICM42688** | BSP/ICM42688/ | SPI2（软, 5MHz） | **已实现（SPI 上下文保护）** |
| **A4950** | BSP/A4950/ | 4 路独立 PWM（快衰减） | **已实现** |
| **GrayADC** | BSP/gray_adc/ | ADC + GPIO（74HC4051） | 已实现 |
| MPU6050 | BSP/MPU6050/ | I2C + EXTI（DMP） | 已实现（预留） |
| JY61P | BSP/JY61P/ | UART | 保留备用 |
| TB6612 | BSP/TB6612/ | PWM + GPIO | 底层保留，注册层已删除 |
| BMP280 | BSP/BMP280/ | I2C | 未启用 |
| QMC5883P | BSP/QMC5883P/ | I2C | 未启用 |
| NRF24L01 | BSP/NRF24L01/ | SPI | 未启用 |

### 6.1 GrayADC — 8 路灰度传感器

- 基于 74HC4051 模拟开关 + 8 路红外对管
- 三根地址线（AD0/AD1/AD2）选通通道 → OUT 模拟电压 → ADC 采集
- 支持校准模式（白/黑基准）→ 二值化 + 归一化
- `GrayADC_LinePosition()`：加权平均法计算黑线位置，EMA 低通滤波
- 3 个打印函数：`GrayADC_PrintRaw` / `PrintBits` / `PrintLinePos`

### 6.2 JY61P — 维特智能六轴陀螺仪（保留备用）

- 通信：UART 主动上报
- 当前主力 IMU 已切换为 ICM42688，JY61P 保留作为 fallback
- 写操作：`JY61P_ZAxisZero()` — Z 轴偏航角归零

### 6.3 ICM42688 — TDK 高性能六轴陀螺仪（当前主力 IMU）

- 通信：SPI2（软, 5MHz DelayOff）— SCK=PA25, MOSI=PB25, MISO=PA24, CS=PB23
- 总线/速率统一由 `SYSTEM/BusRate.h` 配置（`ICM42688_SPI_BUS` / `ICM42688_SPI_SPEED`）
- 默认配置：±16g / ±2000dps / 双 1kHz ODR / 低噪声模式
- ISR 驱动模型：
  - ISR @5ms → `ICM42688_ReadSensor()` — SPI burst 读 + 上下文保护 + float 转换 + 偏航积分
  - 主循环 → `g_icm42688` 全局结构体 / `ICM42688_GetSnapshot()` — 零 SPI
- SPI 上下文保护（防 OLED 冲突）：
  - `ICM42688_BurstReadFast` 内部 `save → SelectBus(SPI2) → burst → restore`
  - 即使主循环正在做 OLED SPI1 刷新，ICM42688 也从 SPI2 正确读取
- 偏航积分基于 `g_sys_tick_ms` 自动计算 dt，首次自动归零
- `ICM42688_ReadSensor` 含 atan2f/sqrtf（M0+ 软件浮点 ~100μs），总耗时 ~120μs

### 6.4 A4950 — 双路 H 桥电机驱动（快衰减 + 单极性 PWM）

- 控制逻辑：
  - 正转：IN1 = PWM, IN2 = 0
  - 反转：IN1 = 0, IN2 = PWM
  - 刹车：IN1 = 1, IN2 = 1（100% 占空比短路制动）
- 4 路独立 PWM（每个 IN 脚一个 PWM 通道），全 PD1 @20kHz, period=4000
- 接口：`API_Motor_SetSpeed(speedA, speedB)` — 正负表示方向，绝对值 = 占空比（≤4000）
- 通过 `API_MOTOR_MAX_DUTY` 宏统一满占空比（当前 4000）
- 安全特性：
  - 上电初始刹车
  - 方向反转自动插入 2ms 刹车死区，防止 H 桥直通
  - 同方向调速直接改占空比，无死区
- TB6612 底层代码保留，注册层已删除

---

## 7. app 应用层 — 三环 PID 控制系统

系统包含三个独立的 PID 控制回路，全部基于 Q16.16 定点整数。

### 7.1 三环 PID 参数总览

| 环 | kp | ki | kd | dt | Out_max | I_max | 死区 | 积分分离 | 用途 |
|------|------|------|------|------|------|------|------|------|------|
| 速度环（左/右） | 20 | 150 | 0 | 20ms | 4000 | 3500 | — | — | 编码器速度→PWM 占空比 |
| 灰度方向环 | 2.0 | 0.5 | 0.1 | 5ms | 4000 | 500 | 60 | 3000 | 线位置偏差→转向舵量 |
| 偏航角环 | 2.0 | 0.3 | 0 | 20ms | 4000 | 500 | 100 | — | 偏航角偏差→转向舵量 |
| 偏航角环（直走） | 0.6 | 0.06 | 0 | 20ms | 1200 | 150 | 200 | — | 直走时温和纠偏 |

**Out_max 约定**：所有 PID 的 Out_max 均使用 `API_MOTOR_MAX_DUTY` 宏（当前 4000），
最终输出由 `MotorOutput_Clamp` 统一限幅到 ±4000。

**速度环** 输出为左右轮独立 PWM 占空比（±4000）。
**方向环 / 偏航角环** 输出为转向舵量，通过混合公式叠加到速度环输出上：
```c
// 方向环融合
left  = speed_out_left  - steer;
right = speed_out_right + steer;

// 偏航角环融合
left  = speed_out_left  + yaw_steer;
right = speed_out_right - yaw_steer;
```

### 7.1.1 方向环（Direction_Control）

- 调用周期：TIMG0 ISR 5ms
- 输入：`GrayADC_LinePosition()` 返回的线位置（0~3500）
- 目标：传感器中心（7 个传感器间距×500μm 的一半）
- 输出：`g_steer`（±4000），暂存供 `LineFollow_Output` 混合
- 丢线（pos<0）时保持上一拍 steer 不变

### 7.1.2 偏航角环（YawPid_Calc）

- 调用周期：TIMG0 ISR 20ms
- 输入：ICM42688 偏航角（度），内部转为 cdeg（×100）
- 含 ±180° 自动回绕处理
- 三种使用模式：
  - `YawTest_Control()` — 纯差速转向测试
  - `Drive_YawSpeed()` — 速度环 + 偏航角环融合
  - `YawPid_InitStraight()` — 直走专用（小舵量 + 宽死区）

### 7.1.3 速度环（PID_EncoderSpeed_Control）

- 调用周期：TIMG0 ISR 20ms
- 输入：编码器 EMA 滤波速度（`Encoder1_Speed` / `Encoder2_Speed`）
- 目标速度由任务层设置（`PID_EncoderSpeed_Set`）
- 左右轮独立 PID，融合方向/偏航 steer 后统一 `MotorOutput_Clamp` 限幅

### 7.2 任务链调度框架（Task_Run）

任务调度框架（[Tasks.c](app/Tasks/Tasks.c) + [Tasks.h](app/Tasks/Tasks.h)），
在 TIMG0 ISR 20ms 插槽或主循环中调用。

**按键协议：**

| 按键 | 功能 | 描述 |
|:---:|------|------|
| **KEY1** | 启动 | 待机时按下 → 锁存当前选中任务号 → 启动 |
| **KEY2** | 选择任务 | 循环切换 `s_task_select` 1→2→3→4→5→6→1（由 KEY.c 维护） |
| **KEY3** | 急停 | 运行中按下 → `Task_Stop()`：停车 + 全部 PID 清零 |
| **KEY4** | 设参 | 循环选择 Task_6 小球目标坐标 -12→+12→-12 |

- 启动瞬间锁存 `s_task_select` → `s_task_active`，运行中 KEY2 不影响当前任务
- `Task_Stop()` 复位：速度环×2 + 方向环 + 偏航角环共 4 个 PID
- 任务实现：
  - `Task_1` — 空壳，预留
  - `Task_2` — 灰度循迹一圈并计时（速度环 + 方向环）
  - `Task_3` — 小球位置控制 0→+5→-5（纯步进电机，不涉及小车）
  - `Task_4` — 灰度循迹 + 小球 X=0（软启动 + 计时触发极缓减速）
  - `Task_5` — 循迹一圈 + 小球 X=0（软启动 + 终点检测 + 极缓减速）
  - `Task_6` — 循迹一圈 + 小球指定坐标（软启动 + 终点检测 + 极缓减速，KEY4 设参）

辅助接口：`Task_IsRunning()` / `Task_GetSelect()` / `Task_GetActive()` / `Task_GetPos()` / `Task_2_GetLapTime()`

### 7.3 任务调度（Control_Task）

**仅使用 TIMG0 一个定时器中断（1ms），所有控制任务在 ISR 中直接执行：**

| 周期 | 任务 | 执行位置 |
|------|------|:---:|
| 1ms | g_sys_tick_ms++, Key_Tick | ISR |
| 5ms | GrayADC_Task + Direction_Control + 帧超时检测 | ISR |
| 20ms | Encoder SnapshotAll + GetFilteredSpeed + Task_Run | ISR |
| 50ms | usart_printf（标志位驱动） | 主循环 |
| 100ms | OLED 刷新（标志位驱动） | 主循环 |

**TaskManager** 仅管理主循环低频任务（`buzzer_5ms` / `key_20ms` / `print_50ms` / `oled_100ms`），在 ISR 中计数+置标志位，主循环轮询消费。


### 7.4 PID 库

Q16.16 定点位置式 PID，特性：
- 浮点 API（`Set_PID` 接受 float，Init 阶段一次性转 Q16.16）
- 纯整数热路径（`PID_Calc` 全程 int32_t / int64_t，< 2µs）
- 死区 I 泄放、积分分离、抗饱和、微分 LPF
- `PID_SetSampleTime` 使用毫秒整数（如 5 → 5ms）
- `PID_Init_WithLimit` 的 Integral_max / Out_max 使用自然单位整数
- `PID_EncoderSpeed_t`：左右轮独立 PID 结构体，同参数同时设置

---

## 8. SYSTEM 与 Core 分工

SYSTEM 层（统一门面）：
- `SYS_Init`：系统初始化入口（含 80MHz 超频配置）。
- `BusRate.h`：总线选择与速率策略。
- `IrqPriority.h`：中断优先级策略（单一时基 + 编码器 + USART 分级）。

Core/MSPM0G3507 层（硬件实现）：
- `G3507_sys.c`：系统时钟与系统信息。
- `G3507_gpio.c` / `G3507_tim.c` / `G3507_usart.c` / `G3507_pwm.c` / `G3507_adc.c`。
- `G3507_exti.c`：外部中断实现。
- `G3507_Encoder.c`：编码器双缓冲架构（EXTI ISR → raw，SnapshotAll → stable）。
- `G3507_soft_i2c.c` / `G3507_soft_spi.c`：软件总线底层（含 SPI 上下文保存/恢复）。

---

## 9. 构建系统与工作流

关键文件：
- `CMakeLists.txt`：G3507-only 构建入口。
- `CMakePresets.json`：Debug/Release 预设。
- `OpenOCD/G3507_OpenOCD.cfg`：烧录配置。

常用命令：
```bash
cmake --preset Debug
cmake --build --preset Debug
```

---

## 10. 开发约定

命名约定：
- `API_xxx_*`：API 层接口。
- `G3507_xxx_*`：Core 层实现。
- `Enroll_xxx_*`：注册层门面函数。

维护约定：
- 业务逻辑不直接碰寄存器。
- 资源变化优先改 Enroll 映射（`G3507_hw_config.h` → `Enroll.c`）。
- 总线和中断策略集中改 SYSTEM 头文件。
- 高频传感器数据优先在 ISR 内读取（ICM42688 直接 SPI burst + 上下文保护），保证数据年龄可控。
- 时序敏感的快照操作（编码器 SnapshotAll）在 ISR 中执行，保证精确等间隔。
- PID 配置 API 接受 float（Init 阶段一次性转换），运行时全部整数。
- **PID Out_max 统一使用 `API_MOTOR_MAX_DUTY` 宏**，改 PWM frequency/period 时只需改一处。
- **ISR 中的 SPI 操作必须使用上下文保护**（`soft_spi_hal_save/restore`），防止抢占主循环的 SPI 事务。
- 非阻塞延时用 `NonBlockDelay_t`，禁止在主循环或 ISR 中使用 `Delay_ms`。
- 串口号通过宏统一管理，避免分散硬编码。
- 编码器速度经 EMA 滤波后再送入 PID（`API_Encoder_GetFilteredSpeed`），`GetSpeed` 用于需原始值的场景。

---

## 11. 快速上手清单

1. README.md
2. docs/arch-guide.md
3. CMakeLists.txt
4. Enroll/Enroll.h
5. Enroll/G3507_hw_config.h
6. A_Entry/main.c
7. SYSTEM/BusRate.h
8. SYSTEM/IrqPriority.h
9. BSP/ICM42688/ICM42688.h
10. BSP/A4950/A4950.h
11. BSP/StepMotor/StepMotor.h（Emm42 V5.0 步进闭环驱动）
12. app/Control/Control.h（三环 PID + 小球位置环接口）
13. app/Control/Control.c（三环 PID 默认参数）
14. app/Tasks/Tasks.h（任务框架 + KEY 协议）
15. API/inc/Encoder.h（编码器 EMA 滤波接口）
16. API/API_SPI/soft_spi_hal.h（SPI 上下文保护接口）
