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
BSP/                    板级设备层（LED/KEY/OLED/ICM42688/AT4950/JY61P/GrayADC 等）
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
- g_sys_tick_ms++ + Key_Tick @1ms（按键消抖，内部 10 分频，30ms 消抖窗口）
- JY61P_Task + **ICM42688_ReadSensor** + GrayADC_Task + 出入线检测 + Direction_Control @5ms
- Encoder Snapshot + GetSpeed + Task_Run @20ms
- TaskManager tick 计数（只置标志位，供主循环消费）

**JY61P_Task + ICM42688_ReadSensor 均在 ISR 5ms 中读取（数据年龄 ≤5ms）。**
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
| 0 | TIMG0 | 系统时基 1ms（ISR 直接执行所有控制任务） |
| 2 | Encoder EXTI / USART2 | 编码器脉冲边沿捕获 / JY61P 陀螺仪 RX |
| 3 | USART1/3/4 + MPU6050 + 缺省 | 调试串口与辅助外设 |

编码器 EXTI 优先级统一在 `SYSTEM/IrqPriority.h` 中管理（`IRQ_PRIO_ENCODER_EXTI 2U`）。

### 4.6 JY61P 数据流架构（当前：ISR 内解析）

JY61P 数据流（当前使用 USART2，115200 bps）：

```text
┌─ USART2 ISR（优先级 2）──────────────────────┐
│  Control_Task_USART_Callback → FIFO 排空     │  ← 硬件 FIFO 循环读出
│  → JY61P_RxPush(byte) → 环形缓冲区           │  ← 只入队，极快
└──────────────────────────────────────────────┘
                    ↓
┌─ TIMG0 ISR 5ms 插槽（优先级 0）─────────────┐
│  JY61P_Task() → 取字节 → 校验 → 整数解析      │  ← 纯整数 cdeg，无浮点
│  JY61P_GetYawFiltered() → 整数 EMA           │  ← 纯整数，无浮点
└──────────────────────────────────────────────┘
                    ↓
┌─ TIMG0 ISR 20ms 插槽（优先级 0）────────────┐
│  YawPid_Calc(yaw_cdeg) → 整数 PID           │  ← Q16.16，无浮点
└──────────────────────────────────────────────┘
```

设计原则：
- USART4 ISR 只做最轻量的数据搬运（硬件 FIFO 排空 + push 到环形缓冲）。
- TIMG0 ISR 5ms 中完成数据包解析和 EMA 滤波，全程纯整数（int32_t cdeg）。
  - 解析产出的 `yaw_cdeg = raw × 18000 / 32768`（一次 int64 乘除，< 100ns）。
  - EMA 滤波 `filtered += diff × 3 / 10`（纯整数，< 200ns）。
- TIMG0 ISR 20ms 中 YawPid_Calc 直接接收 cdeg，零转换开销。
- 整个热路径无 float/double，M0+ 无 FPU 开销。ISR 最坏执行时间 < 60µs。

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

### 5.1 PWM 多定时器支持（5 路）

`G3507_pwm.c` 统一支持 **5 种定时器**（TIMG7/TIMG6/TIMA0/TIMA1/TIMG8），8 个 PWM 通道：

| API | 通道 | 引脚 | 硬件定时器 | CCP | 电源域 | 时钟 | Period | 频率 | 用途 |
|-----|------|------|-----------|-----|--------|------|--------|------|------|
| TIM1 | CH1 | PB15 | TIMG7 | CCP0 | PD1 | 80M | 4000 | 20k | AT4950 AIN1 |
| TIM2 | CH1 | PB7 | TIMG6 | CCP1 | PD1 | 80M | 4000 | 20k | AT4950 AIN2 |
| TIM3 | CH1 | PB13 | TIMA0 | CCP3 | PD1 | 80M | 4000 | 20k | AT4950 BIN1 |
| TIM3 | CH2 | PB9 | TIMA0 | CCP1 | PD1 | 80M | 4000 | 20k | AT4950 BIN2 |
| TIM4 | CH1 | PA28 | TIMA1 | CCP0 | PD1 | 80M | 4000 | 20k | 预留 |
| TIM4 | CH2 | PA31 | TIMA1 | CCP1 | PD1 | 80M | 4000 | 20k | 预留 |
| TIM5 | CH1 | PA29 | TIMG8 | CCP0 | PD0 | 40M | 2000 | 20k | 预留 |
| TIM5 | CH2 | PA30 | TIMG8 | CCP1 | PD0 | 40M | 2000 | 20k | 预留 |

- `API_PWM_CORE_CCP0~3`：4 个比较通道宏，TIMA0 使用 CCP3（PB13）
- `G3507_PWM_IsTimerG`：自动识别 TIMG 类（TIMG6/7/8）vs TIMA 类（TIMA0/1），分支到对应 API
- `HW_PWM_MAP` 中改 `coreTimId` + `coreChannel` 即可重新分配引脚，底层全自动适配
- 输出极性：LACT LOW / CDACT HIGH（边缘对齐减计数）→ `duty=0` 恒低，`duty=period` 恒高（刹车用）

---

## 6. BSP 层器件

| 模块 | 路径 | 接口类型 | 状态 |
|------|------|----------|------|
| LED | BSP/LED/ | GPIO | 已实现 |
| KEY | BSP/KEY/ | GPIO（消抖） | 已实现 |
| OLED | BSP/OLED/ | SPI（软） | 已实现 |
| MPU6050 | BSP/MPU6050/ | I2C + EXTI（DMP） | 已实现（预留） |
| **ICM42688** | BSP/ICM42688/ | SPI2（软, 5MHz） | **已实现（ISR 驱动）** |
| **AT4950** | BSP/AT4950/ | 4 路独立 PWM（快衰减） | **已实现** |
| **GrayADC** | BSP/gray_adc/ | ADC + GPIO（74HC4051） | 已实现 |
| **JY61P** | BSP/JY61P/ | UART（USART2, 115200 bps） | 已实现 |
| TB6612 | BSP/TB6612/ | PWM + GPIO | 底层保留，注册层已删除 |

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

### 6.3 ICM42688 — TDK 高性能六轴陀螺仪（当前主力 IMU）

- 通信：SPI2（软, 5MHz DelayOff）— SCK=PA25, MOSI=PB25, MISO=PA24, CS=PB23
- 总线/速率统一由 `SYSTEM/BusRate.h` 配置（`ICM42688_SPI_BUS` / `ICM42688_SPI_SPEED`）
- 默认配置：±16g / ±2000dps / 双 1kHz ODR / 低噪声模式
- ISR 驱动模型：
  - ISR @5ms → `ICM42688_ReadSensor()` — 一次 12 字节 SPI burst + float 转换 + 偏航积分
  - 主循环 / 控制代码 → `ICM42688_GetAttitude()` / `GetGyroscope()` / `GetAccelerometer()` — 零 SPI
- 偏航积分基于 `g_sys_tick_ms` 自动计算 dt，首次自动归零
- `ICM42688_ReadSensor` 含 atan2f/sqrtf（M0+ 软件浮点 ~100μs），总耗时 ~120μs
- 读数据接口对标 MPU6050 风格：`GetAttitude(&roll, &pitch, &yaw)` / `GetGyroscope()` / `GetAccelerometer()`

### 6.4 AT4950 — 双路 H 桥电机驱动（快衰减 + 单极性 PWM）

- 控制逻辑：
  - 正转：IN1 = PWM, IN2 = 0
  - 反转：IN1 = 0, IN2 = PWM
  - 刹车：IN1 = 1, IN2 = 1（100% 占空比短路制动）
- 4 路独立 PWM（每个 IN 脚一个 PWM 通道），全 PD1 @20kHz, period=4000
- 接口：`AT4950_SetSpeed(speedA, speedB)` — 正负表示方向，绝对值 = 占空比
- 安全特性：
  - 上电初始刹车（`AT4950_Init`）
  - 方向反转自动插入 2ms 刹车死区，防止 H 桥直通
  - 同方向调速直接改占空比，无死区
- TB6612 底层代码保留，注册层已删除

---

## 7. app 应用层

### 7.1 循线控制（Control）

状态机三态：直走循线 → 路口等待 → 差速转弯

转弯参数：
- `TURN_DELAY_MS`：看到路口后等待多久再转
- `TURN_PIVOT_MS`：差速转弯持续时长
- `INTERSECTIONS_PER_LAP`：每圈路口数（= 4）

PID（Q16.16 整数）：
- 速度环（20ms）：左右轮独立 PID，输出限幅 ±TB6612_MAX_DUTY
- 方向环（5ms）：灰度线位置 PID，输出限幅 1500（2000 占空比标度），死区 60，带积分分离
- 配置 API 接受 float（如 `Set_PID(&pid, 0.1f, 0.0004f, 0.005f)`），内部一次性转 Q16.16
- ISR 热路径 `PID_Calc` 全程纯整数，耗时 < 2µs @80MHz
- `PID_SetDeadband` / `PID_SetIntegralSeparation` 等接口使用自然单位整数

### 7.1.1 任务链调度（Task_Run）

比赛任务框架（[Tasks.c](app/Tasks/Tasks.c)），与循线主控 `Control_Run` 并列（20ms ISR 插槽挂 `Task_Run`，`Control_Run` 注释保留）：

- **KEY1**：待机时按下 → 锁存当前任务号并启动；运行时按下 → 急停（`Task_Stop`：停车 + PID 全复位）
- **KEY2**：循环选择任务 1→2→3→4→1（`s_task_select` 由 KEY.c 维护）
- **KEY3**：未使用
- 启动瞬间锁存任务号到 `s_task_active`，运行中按 KEY2 不影响当前任务

| 任务 | 描述 |
|:---:|------|
| Task_1 | 直走遇线停车（yaw 0° + 速度环 → 入线停车） |
| Task_2 | 循迹一圈 A→B→C→D→A（yaw 直走 + 灰度巡线交替） |
| Task_3 | 交叉循迹一圈 A→C→B→D→A（开环旋转 + yaw 直走 + 灰度巡线） |
| Task_4 | 同 Task_3 轨迹，自动行驶 4 圈后停车（复用 `Task34_Run(max_laps)`） |

`Task_3`/`Task_4` 共用 `static Task34_Run(max_laps)` 状态机，零代码重复。

辅助接口：`Task_IsRunning()` / `Task_GetSelect()` / `Task_GetActive()` / `Task_GetPos()`

### 7.2 任务调度（Control_Task）

**仅使用 TIMG0 一个定时器中断（1ms），所有控制任务在 ISR 中直接执行：**

| 周期 | 任务 | 执行位置 |
|------|------|:---:|
| 1ms | g_sys_tick_ms++, Key_Tick（内部 10 分频，30ms 消抖窗口） | ISR |
| 5ms | JY61P_Task + **ICM42688_ReadSensor** + GrayADC_Task + 出入线检测 + Direction_Control | ISR |
| 20ms | Encoder Snapshot + GetSpeed + Task_Run | ISR |
| 50ms | usart_printf（标志位驱动） | 主循环 |
| 100ms | OLED 刷新（标志位驱动） + AT4950 测试（500ms 切换） | 主循环 |

**TaskManager** 仅管理主循环低频任务（`buzzer_5ms` / `key_20ms` / `print_50ms` / `oled_100ms`），在 ISR 中计数+置标志位，主循环轮询消费。

ISR 5ms 槽耗时 ~185µs（占 3.7% CPU）：GrayADC(40μs) + JY61P(5μs) + ICM42688(120μs) + Direction_PID(15μs)。

USART 中断回调：
- USART2：`JY61P_RxPush()` — 仅环形缓冲入队
- USART4：预留

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
- 高频传感器数据优先在 ISR 内读取（JY61P 环形缓冲 / ICM42688 直接 SPI burst），保证数据年龄可控。浮点计算量大的融合/滤波放主循环下半部。
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
9. BSP/ICM42688/ICM42688.h
10. BSP/AT4950/AT4950.h
