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
BSP/                    板级设备层（LED/KEY/OLED/MPU6050/TB6612/GrayADC/JY61P 等）
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

### 4.3 软件总线双分层（I2C/SPI）

- API 协议层：负责起始、停止、收发字节等协议流程。
- Core 底层层：负责 GPIO 翻转和延时。

### 4.4 单一时基前后台架构（TIMG0 1ms）

**仅使用一个定时器 TIMG0** 作为系统时基（1ms 中断）。

ISR 中直接执行时序敏感的控制任务：
- Key_Tick @1ms（按键消抖，内部 20 分频）
- GrayADC_Task + Direction_Control @5ms
- Encoder Snapshot + GetSpeed + Control_Run @20ms
- TaskManager tick 计数（只置标志位，供主循环消费）

**所有 PID 运算使用 Q16.16 整数，ISR 最坏执行时间 < 60µs**。

主循环仅负责非实时任务：
- JY61P_Task（环形缓冲解析）
- key_Get（按键事件同步）
- OLED 刷新（100ms，由 TaskManager 标志位驱动）
- printf 打印（50ms，由 TaskManager 标志位驱动）

### 4.5 统一中断优先级策略

统一策略放在 `SYSTEM/IrqPriority.h`。M0+ 仅 4 级（0~3）：

| 优先级 | 外设 | 说明 |
|:---:|------|------|
| 0 | TIMG0 | 系统时基 1ms（ISR 直接执行所有控制任务） |
| 2 | Encoder EXTI / USART4 | 编码器脉冲边沿捕获 / JY61P 陀螺仪 RX |
| 3 | USART1/2/3 + MPU6050 + 缺省 | 调试串口与辅助外设 |

编码器 EXTI 优先级硬编码在 `G3507_Encoder.c` 中（本地宏 `G3507_ENCODER_EXTI_PRIO 2U`）。

### 4.6 中断上半部/下半部分离（ISR 只收发，不解析）

JY61P 数据流（当前使用 USART4）：

```text
┌─ USART4 ISR（优先级 3）─────────────┐
│  JY61P_RxPush(byte) → 环形缓冲区    │  ← 极快，只入队
└─────────────────────────────────────┘
                 ↓
┌─ 主循环 while(1) ──────────────────┐
│  JY61P_Task() → 取字节 → 校验 → 浮点  │  ← 不在 ISR 上下文
└─────────────────────────────────────┘
```

设计原则：
- ISR 只做最轻量的数据搬运（push 到环形缓冲）。
- 状态机解析 + 浮点运算放在主循环（80MHz，Cortex-M0+ 无硬件 FPU）。
- ISR 内做浮点会显著增加中断延迟，阻塞其他中断。

### 4.7 非阻塞延时

基于 TIMG0 1ms 全局 tick（`g_sys_tick_ms`）的纯整数延时系统：

```c
NonBlockDelay_t d;
NonBlockDelay_Start(&d, 200);        // 启动 200ms 延时，立刻返回
if (NonBlockDelay_IsDone(&d)) { ... } // 轮询是否到期

LED_TurnNb_Start(Buzzer1, 200);      // 非阻塞蜂鸣器（替代阻塞版 LED_Turn）
LED_TurnNb_Task();                   // 主循环调用，超时自动关
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
LED_TurnNb_Start(Buzzer1, 200);  // 响 200ms，立刻返回
LED_TurnNb_Task();               // 主循环调用，超时自动关

// 多通道独立计时
LED_TurnNb_Start(Buzzer1, 200);
LED_TurnNb_Start(LED1, 500);
```

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
| API/inc/Encoder.h | 编码器接口 | 已实现（EXTI 软件编码器） |
| API/API_I2C/API_I2C.h | 软件 I2C 协议 | 已实现 |
| API/API_SPI/API_SPI.h | 软件 SPI 协议 | 已实现 |

---

### 5.1 PWM 双定时器支持

`G3507_pwm.c` 同时支持 TIMA1（高级定时器）和 TIMG8（通用定时器）：

| 方案 | 引脚 | 定时器 | hw_config.h 中 coreTimId |
|------|------|--------|--------------------------|
| A（当前 PCBV3.0） | PB15=CCP0→PWMA, PB7=CCP1→PWMB | TIMG8 | `API_PWM_CORE_TIMG8` |
| B（旧板 PCBV1.0） | PA16=CCP1, PA17=CCP0 | TIMA1 | `API_PWM_CORE_TIMA1` |

`HW_PWM_MAP` 中改 `coreTimId` 即可切换，`ConfigPin`/`InitTimer`/`SetCCR` 自动适配 DL_TimerG_* 或 DL_TimerA_* API。

PWM 参数（当前）：

- **频率 20kHz**，占空比 **0-2000**（每步 0.05%）：`API_PWM_Init(API_PWM_TIM1, 2000-1, 1-1)`
- TIMG8 在 **PD0 电源域**，时钟为 ULPCLK 40MHz（不是 80MHz MCLK）：40M ÷ 1 ÷ 2000 = 20kHz
- 输出极性：装载事件拉低 / 下行比较拉高，**duty=0 时 CC=period 比较永不触发 → 输出纯恒低**
  （历史教训：旧极性 duty=0 会输出恒高 100%，导致电机"给 0 停、给任意值满转"）
- `TB6612_MAX_DUTY = 2000` 与 ARR+1 保持一致，改 PWM 分辨率必须同步等比缩放
  PID 增益、PID 输出限幅与所有写死的占空比常数

---

## 6. BSP 层器件

| 模块 | 路径 | 接口类型 | 状态 |
|------|------|----------|------|
| LED | BSP/LED/ | GPIO | 已实现 |
| KEY | BSP/KEY/ | GPIO（消抖） | 已实现 |
| OLED | BSP/OLED/ | I2C/SPI | 已实现 |
| MPU6050 | BSP/MPU6050/ | I2C + EXTI（DMP） | 已实现（DMP 已启用） |
| TB6612 | BSP/TB6612/ | PWM + GPIO | 已实现（PWM 默认 TIMG8） |
| **GrayADC** | BSP/gray_adc/ | ADC + GPIO（74HC4051） | 已实现 |
| **JY61P** | BSP/JY61P/ | UART（USART4, 115200 bps） | 已实现 |

### 6.1 GrayADC — 8 路灰度传感器

- 基于 74HC4051 模拟开关 + 8 路红外对管
- 三根地址线（AD0/AD1/AD2）选通通道 → OUT 模拟电压 → ADC 采集
- 支持校准模式（白/黑基准）→ 二值化 + 归一化
- `GrayADC_LinePosition()`：加权平均法计算黑线位置，EMA 低通滤波
- 3 个打印函数：`GrayADC_PrintRaw` / `PrintBits` / `PrintLinePos`

### 6.2 JY61P — 维特智能六轴陀螺仪

- 通信：UART 主动上报（默认 9600 bps，可用上位机改 115200）
- 协议：0x55 + 类型（0x51/0x52/0x53）+ 8 字节数据 + 校验和
- 三种数据包：加速度（±16g）、角速度（±2000°/s）、欧拉角（±180°）
- 中断上半部/下半部分离架构
- 写操作：`JY61P_ZAxisZero()` — Z 轴偏航角归零
- 输出速率和波特率通过上位机配置（传感器内部 MCU 保存）

---

## 7. app 应用层

### 7.1 循线控制（Control）

状态机三态：直走循线 → 路口等待 → 差速转弯

转弯参数：
- `TURN_DELAY_MS`：看到路口后等待多久再转
- `TURN_PIVOT_MS`：差速转弯持续时长
- `INTERSECTIONS_PER_LAP`：每圈路口数（= 4）

圈数控制：
- KEY3 设定目标圈数（1-5 循环），`s_target_laps` 由 KEY.c 维护
- `s_intersection_count` 每过一个路口 +1
- 达到 `s_target_laps × 4` 后自动停车
- `s_need_white` 防重复计数：转弯结束后必须先见白才能检测下一个路口
- Key 消费模式：每次按键处理后 `Key = 0U`，防止全局变量持久化导致重复触发

PID（Q16.16 整数）：
- 速度环（20ms）：左右轮独立 PID，输出限幅 ±TB6612_MAX_DUTY
- 方向环（5ms）：灰度线位置 PID，输出限幅 1500（2000 占空比标度），死区 60，带积分分离
- 配置 API 接受 float（如 `Set_PID(&pid, 0.1f, 0.0004f, 0.005f)`），内部一次性转 Q16.16
- ISR 热路径 `PID_Calc` 全程纯整数，耗时 < 2µs @80MHz
- `PID_SetDeadband` / `PID_SetIntegralSeparation` 等接口使用自然单位整数

### 7.1.1 任务链调度（Task_Run）

比赛任务框架，与循线主控 `Control_Run` 并列（当前 20ms ISR 插槽挂 `Task_Run`，`Control_Run` 注释保留）：

- **KEY1**：待机时按下 → 锁存当前任务号并启动；运行时按下 → 急停（`Task_Stop`：停车 + PID 全复位）
- **KEY2**：循环选择任务 1→2→3→4→1（`s_task_select` 由 KEY.c 维护，与 KEY3 设圈数同款消费模式）
- 启动瞬间锁存任务号到 `s_task_active`，运行中按 KEY2 不影响当前任务
- 任务体 `Task_1..4`（Control.c），签名内可用左右轮编码器速度
- 辅助接口：`Task_IsRunning()` / `Task_GetSelect()` / `Task_GetActive()`（OLED 显示用）

### 7.2 任务调度（Control_Task）

**仅使用 TIMG0 一个定时器中断（1ms），所有控制任务在 ISR 中直接执行：**

| 周期 | 任务 | 执行位置 |
|------|------|:---:|
| 1ms | Key_Tick（内部 10 分频，30ms 消抖窗口） | ISR |
| 5ms | GrayADC_Task + Direction_Control | ISR |
| 20ms | Encoder Snapshot + GetSpeed + Task_Run（Control_Run 保留） | ISR |
| 50ms | usart_printf（标志位驱动） | 主循环 |
| 100ms | OLED 刷新（标志位驱动） | 主循环 |

**TaskManager** 仅管理主循环低频任务（`print_50ms` / `oled_100ms`），在 ISR 中计数+置标志位，主循环轮询消费。

ISR 最坏执行时间（Q16.16 PID）：~60µs @80MHz（占 1ms 周期的 6%）。

USART 中断回调：
- USART4：`JY61P_RxPush()` — 仅环形缓冲入队（JY61P 串口号可通过 `JY61P_USART` 宏统一切换）

### 7.3 PID 库

Q16.16 定点位置式 PID，特性：
- 浮点 API（`Set_PID` 接受 float，Init 阶段一次性转 Q16.16）
- 纯整数热路径（`PID_Calc` 全程 int32_t / int64_t，< 2µs）
- 死区 I 泄放、积分分离、抗饱和、微分 LPF
- `PID_SetSampleTime` 使用毫秒整数（如 5 → 5ms）
- `PID_Init_WithLimit` 的 Integral_max / Out_max 使用自然单位整数

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
- `G3507_soft_i2c.c` / `G3507_soft_spi.c`：软件总线底层。

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
- 高频传感器数据不在 ISR 内解析，用环形缓冲 + 主循环下半部。
- 时序敏感的快照操作（编码器 SnapshotAll）在 ISR 中执行，保证精确等间隔。
- PID 配置 API 接受 float（Init 阶段一次性转换），运行时全部整数。
- 非阻塞延时用 `NonBlockDelay_t`，禁止在主循环或 ISR 中使用 `Delay_ms`。
- 串口号通过宏统一管理（如 `JY61P_USART`），避免分散硬编码。

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
9. app/Control/Control.c
10. app/Control_Task/Control_Task.c
