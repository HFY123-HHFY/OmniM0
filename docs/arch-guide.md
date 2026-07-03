# OmniLayer 工程架构深度解析（G3507 单平台）

> 本文档用于快速恢复对 OmniLayer 当前架构的完整认知。
> 当前仓库已收敛为单 MCU 目标：TI MSPM0G3507。

---

## 1. 项目元信息

| 项目 | 详情 |
|------|------|
| 名称 | OmniLayer |
| 定位 | 面向电赛场景的 G3507 分层开发框架 |
| 构建工具 | CMake + GCC ARM Embedded + OpenOCD |
| IDE 兼容 | VS Code（主）+ Keil MDK（兼容保留） |
| 主维护方向 | 裸机主线 + 后续 RTOS 演进 |
| 作者 | Hu Fangyuan |
| 联系方式 | 634591772@qq.com |

---

## 2. 目标平台

| MCU | 架构 | 内核 | 构建预设 |
|-----|------|------|----------|
| TI MSPM0G3507 | ARM | Cortex-M0+ | Debug / Debug-G3507 |

说明：
- 当前仓库为 G3507-only，不再维护多 MCU 选择分支。
- ENROLL_MCU_TARGET 固定为 ENROLL_MCU_G3507。

---

## 3. 分层架构总览

```text
A_Entry/                程序入口（main.c）
app/                    应用层（控制逻辑、任务调度、算法）
BSP/                    板级设备层（LED/KEY/OLED/MPU6050/TB6612 等）
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

数据流：

```text
G3507_hw_config.h (映射宏)
  -> Enroll.c (X-Macro 展开配置表)
  -> Enroll_xxx_Register()
  -> API/BSP Register()
  -> 运行期按逻辑 ID 使用资源
```

收益：
- 业务层不关心具体引脚。
- 新增板级资源时，优先改映射表与注册逻辑。

### 4.2 两阶段初始化（Register -> Init）

- 阶段 1：Enroll_xxx_Register() 登记配置表。
- 阶段 2：API_xxx_Init() 激活硬件。

这种方式把“资源描述”与“硬件激活”分离，调试更清晰。

### 4.3 软件总线双分层（I2C/SPI）

- API 协议层：负责起始、停止、收发字节等协议流程。
- Core 底层层：负责 GPIO 翻转和延时。

桥接头文件：
- API/API_I2C/soft_i2c_hal.h
- API/API_SPI/soft_spi_hal.h

BSP 只调用 API_I2C/API_SPI，不直接触碰 Core。

### 4.4 统一中断优先级策略

统一策略放在 SYSTEM/IrqPriority.h：
- 策略层决定谁高谁低。
- 机制层负责写 NVIC。

当前典型优先级：
- TIM 控制节拍最高。
- MPU6050 与编码器为高实时输入。
- USART 低于控制链路。

---

## 5. API 层外设接口（当前）

| API 头文件 | 功能 | G3507 状态 |
|-----------|------|------------|
| API/inc/gpio.h | GPIO 输入输出 | 已实现 |
| API/inc/usart.h | 串口通信 | 已实现 |
| API/inc/pwm.h | PWM 输出 | 已实现 |
| API/inc/tim.h | 定时器中断 | 已实现 |
| API/inc/adc.h | ADC 采集 | 已实现 |
| API/inc/exti.h | 外部中断 | 已实现 |
| API/inc/Encoder.h | 编码器接口 | 已实现（EXTI 软件编码器） |
| API/API_I2C/API_I2C.h | 软件 I2C 协议 | 已实现 |
| API/API_SPI/API_SPI.h | 软件 SPI 协议 | 已实现 |

---

## 6. BSP 层器件（当前常用）

| 模块 | 文件 | 接口类型 |
|------|------|----------|
| LED | BSP/LED/LED.c | GPIO |
| KEY | BSP/KEY/KEY.c | GPIO（消抖） |
| OLED | BSP/OLED/OLED.c | I2C/SPI |
| MPU6050 | BSP/MPU6050/MPU6050.c | I2C + EXTI |
| MPU6050 DMP | BSP/MPU6050/eMPL/ | 官方库适配 |
| TB6612 | BSP/TB6612/TB6612.c | PWM + GPIO |

说明：
- 文档仅列当前主流程使用的模块。
- 其余 BSP 目录可按实际需求启用或裁剪。

---

## 7. SYSTEM 与 Core 分工

SYSTEM 层（统一门面）：
- SYS_Init：系统初始化入口。
- SYS_EXTI_GetIrqn / SYS_EXTI_GetLineIndex：EXTI 辅助映射。
- BusRate.h：总线选择与速率策略。
- IrqPriority.h：中断优先级策略。

Core/MSPM0G3507 层（硬件实现）：
- G3507_sys.c：系统时钟与系统信息。
- G3507_exti.c：外部中断实现。
- G3507_gpio.c / G3507_tim.c / G3507_usart.c / G3507_pwm.c / G3507_adc.c。
- G3507_soft_i2c.c / G3507_soft_spi.c：软件总线底层。

---

## 8. 构建系统与工作流

关键文件：
- CMakeLists.txt：G3507-only 构建入口。
- CMakePresets.json：Debug/Release 预设，目标固定 G3507。
- OpenOCD/G3507_OpenOCD.cfg：烧录配置。
- .vscode/tasks.json：Build/Flash 任务。
- .vscode/keybindings.json：F7/F8 快捷键。

常用命令：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

构建产物：
- build/Debug/artifacts/OmniLayer_MSPM0G3507.elf
- 同目录下 .bin / .hex

---

## 9. 开发约定

命名约定：
- API_xxx_*：API 层接口。
- G3507_xxx_*：Core 层实现。
- Enroll_xxx_*：注册层门面函数。

维护约定：
- 业务逻辑不直接碰寄存器。
- 资源变化优先改 Enroll 映射。
- 总线和中断策略集中改 SYSTEM 头文件。

---

## 10. 快速上手清单

建议按以下顺序阅读：
1. README.md
2. docs/arch-guide.md
3. CMakeLists.txt
4. Enroll/Enroll.h
5. Enroll/G3507_hw_config.h
6. A_Entry/main.c
7. SYSTEM/BusRate.h
8. SYSTEM/IrqPriority.h
9. API/src/*.c
10. Core/MSPM0G3507/src/*.c

---

## 11. 近期状态

- 仓库已完成单平台化，当前仅维护 G3507。
- 构建链路保持 GCC + CMake + OpenOCD。
- 分层架构保持不变，重点演进 API/BSP/Enroll 的可维护性与可教学性。
