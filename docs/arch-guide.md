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

### 4.4 统一中断优先级策略

统一策略放在 `SYSTEM/IrqPriority.h`：
- 策略层决定谁高谁低。
- 机制层负责写 NVIC。
- **每个外设实例独立分配优先级**，不再所有 USART 共用一个宏。

当前优先级分配（M0+ 仅 4 级：0~3）：

| 优先级 | 外设 | 说明 |
|:---:|------|------|
| 0 | TIM1 (TIMG0) | 控制节拍 1ms（方向环 5ms + 灰度采集） |
| 1 | TIM2 (TIMG6) | 编码器 20ms（速度环 + Control_Run） |
| 2 | USART3 | JY61P 陀螺仪实时数据流，需低延迟 |
| 3 | USART1/2/4 + 缺省 | 调试串口、MPU6050、编码器 EXTI 等 |

### 4.5 中断上半部/下半部分离（ISR 只收发，不解析）

JY61P 数据流：

```text
┌─ USART3 ISR（优先级 2）─────────────┐
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

### 4.6 MSPM0 UART FIFO 与中断触发

MSPM0G3507 的 UART 有 **4 字节硬件 RX FIFO**，默认中断阈值 = 半满（2 字节）。

**重要**：必须把阈值设为 1 字节，否则帧尾字节会卡在 FIFO 里，直到下一帧到达才释放——这是数据卡顿的根本原因之一。

```c
// G3507_usart.c — 所有 UART 实例初始化时统一设置
DL_UART_Main_setRXFIFOThreshold(map.regs,
                                DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
```

---

## 5. API 层外设接口（当前）

| API 头文件 | 功能 | G3507 状态 |
|-----------|------|------------|
| API/inc/gpio.h | GPIO 输入输出 | 已实现 |
| API/inc/usart.h | 串口通信 | 已实现（4 路，独立优先级） |
| API/inc/pwm.h | PWM 输出 | 已实现 |
| API/inc/tim.h | 定时器中断 | 已实现（独立优先级） |
| API/inc/adc.h | ADC 采集 | 已实现 |
| API/inc/exti.h | 外部中断 | 已实现 |
| API/inc/Encoder.h | 编码器接口 | 已实现（EXTI 软件编码器） |
| API/API_I2C/API_I2C.h | 软件 I2C 协议 | 已实现 |
| API/API_SPI/API_SPI.h | 软件 SPI 协议 | 已实现 |

---

## 6. BSP 层器件

| 模块 | 路径 | 接口类型 | 状态 |
|------|------|----------|------|
| LED | BSP/LED/ | GPIO | 已实现 |
| KEY | BSP/KEY/ | GPIO（消抖） | 已实现 |
| OLED | BSP/OLED/ | I2C/SPI | 已实现 |
| MPU6050 | BSP/MPU6050/ | I2C + EXTI | 已实现（已注释，后续启用） |
| TB6612 | BSP/TB6612/ | PWM + GPIO | 已实现 |
| **GrayADC** | BSP/gray_adc/ | ADC + GPIO（74HC4051） | **新增** |
| **JY61P** | BSP/JY61P/ | UART（9600/115200） | **新增** |

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
- KEY4 设定目标圈数（1-5 循环），`s_target_laps` 由 KEY.c 维护
- `s_intersection_count` 每过一个路口 +1
- 达到 `s_target_laps × 4` 后自动停车
- `s_need_white` 防重复计数：转弯结束后必须先见白才能检测下一个路口
- Key 消费模式：每次按键处理后 `Key = 0U`，防止全局变量持久化导致重复触发

PID：
- 速度环（20ms）：左右轮独立 PID，输出限幅 ±TB6612_MAX_DUTY
- 方向环（5ms）：灰度线位置 PID，输出限幅 180，死区 60，带积分分离
- PID_SetTarget(target) + PID_Calc(actual)：error = target - actual
- 方向环输出取反后叠加到速度环差速输出

### 7.2 任务调度（Control_Task）

| 定时器 | 周期 | 任务 |
|--------|------|------|
| TIM1 (1ms → 5ms) | 5ms | GrayADC_Task + Direction_Control（转弯时跳过） |
| TIM2 (1ms → 20ms) | 20ms | 编码器快照 + Control_Run |
| TIM2 (1ms → 50ms) | 50ms | print_task_flag 置位 |

USART 中断回调：
- USART1：上位机数据包解析
- USART3：`JY61P_RxPush()` — 仅环形缓冲入队

### 7.3 PID 库

位置式 PID，特性：死区 I 泄放、积分分离、抗饱和、微分 LPF。

---

## 8. SYSTEM 与 Core 分工

SYSTEM 层（统一门面）：
- `SYS_Init`：系统初始化入口（含 80MHz 超频配置）。
- `BusRate.h`：总线选择与速率策略。
- `IrqPriority.h`：中断优先级策略（按实例独立分配）。

Core/MSPM0G3507 层（硬件实现）：
- `G3507_sys.c`：系统时钟与系统信息。
- `G3507_gpio.c` / `G3507_tim.c` / `G3507_usart.c` / `G3507_pwm.c` / `G3507_adc.c`。
- `G3507_exti.c`：外部中断实现。
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
- 资源变化优先改 Enroll 映射。
- 总线和中断策略集中改 SYSTEM 头文件。
- 高频传感器数据不在 ISR 内解析，用环形缓冲 + 主循环下半部。

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
