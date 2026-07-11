# OmniM0 — 电赛控制工程 ⚡

基于 [OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git) 分层架构构建的 TI MSPM0G3507 单平台嵌入式项目。

## 🚀 项目定位

OmniM0 = **OmniLayer 架构 × 电赛M0+内核 控制场景**，目标是把工程做成可维护、可扩展、可长期迭代的控制系统底座。

- ✅ 当前仅维护 **MSPM0G3507** 单 MCU，开发路径更聚焦
- ✅ 保留 OmniLayer 的 **GCC + CMake + OpenOCD** 全工具链
- ✅ 保留分层架构思想：应用逻辑与芯片实现解耦
- ✅ 为后续 RTOS 演进与模块复用预留空间

## ✨ 架构亮点

- 🧱 **分层清晰** — A_Entry / app / BSP / Enroll / API / Core / SYSTEM / Drivers 职责分明
- ⚙️ **单平台收敛** — 工程已切换为 G3507-only，减少无效分支维护成本
- 🧩 **注册层（Enroll）** — X-Macro 编译期映射，统一资源注册入口
- 🚌 **软件总线** — I2C/SPI 协议层与底层 GPIO 翻转分离，速率集中配置
- 🎯 **统一中断策略** — 优先级集中在 SYSTEM/IrqPriority.h 管理
- 🔧 **统一构建烧录** — CMake Presets + OpenOCD，构建与下载流程稳定

## 🧩 注册层（Enroll）是什么

`Enroll/` 可以理解为"硬件资源注册中心"：

- 把板级资源映射到具体端口、引脚、外设实例
- 把上层逻辑 ID 绑定到底层驱动能力
- 让 app/BSP 尽量不直接依赖寄存器细节

一句话：**先注册，再初始化，再调用**。

## ⚙️ 平台与工具链

| 项目 | 说明 |
|------|------|
| 目标 MCU | TI MSPM0G3507 |
| 编译链 | GCC ARM Embedded |
| 构建系统 | CMake + Ninja |
| 烧录工具 | OpenOCD |
| 主工作流 | VS Code |

## 📁 项目结构

```text
OmniM0/
├─ A_Entry/                    # 程序入口 (main.c)
├─ app/                        # 应用层
│  ├─ Control/                 # 循线控制（PID+转弯+圈数）
│  ├─ Control_Task/            # 任务调度（TIM1/TIM2/USART 回调）
│  ├─ PID/                     # 位置式 PID + 编码器速度环
│  ├─ Filter/                  # 滤波器
│  └─ My_Usart/                # 串口 printf 重定向 + 数据包解析
├─ BSP/                        # 板级设备层
│  ├─ LED/  KEY/  OLED/
│  ├─ MPU6050/                 # 六轴姿态（已注释，后续启用）
│  ├─ TB6612/                  # 电机驱动（±400 占空比）
│  ├─ gray_adc/                # ★ 8 路灰度传感器（74HC4051 模拟开关）
│  └─ JY61P/                   # ★ 维特智能六轴陀螺仪（UART 主动上报）
│  ├─ BMP280/                  # 可选模块（按需启用）
│  ├─ QMC5883P/                # 可选模块（按需启用）
│  └─ NRF24L01/                # 可选模块（按需启用）
├─ API/                        # 片内外设抽象层 + 协议层
│  ├─ inc/ src/                # gpio/adc/pwm/tim/usart/exti/encoder
│  ├─ API_I2C/                 # 软件 I2C 协议层
│  └─ API_SPI/                 # 软件 SPI 协议层
├─ Enroll/                     # ★ 硬件资源注册中心
├─ Core/
│  └─ MSPM0G3507/              # G3507 底层实现（sys/gpio/tim/usart/adc/pwm/exti）
├─ SYSTEM/                     # 系统层：sys/Delay/BusRate/IrqPriority
├─ Drivers/
│  └─ Drivers_M0G3507/         # TI DriverLib + CMSIS + 启动文件
├─ OpenOCD/
│  └─ G3507_OpenOCD.cfg        # 下载配置
├─ docs/
│  └─ arch-guide.md            # 架构深度解析
├─ CMakeLists.txt
├─ CMakePresets.json
└─ gcc-arm-none-eabi.cmake
```

## 🏗️ 分层数据流

```text
A_Entry/main.c
   ↓
app (业务逻辑/控制算法)
   ↓
BSP (设备封装)
   ↓
API (统一外设接口 + I2C/SPI 协议)
   ↓
Core/MSPM0G3507 (硬件寄存器与 DriverLib 实现)
```

注册路径：

```text
G3507_hw_config.h -> Enroll.c -> API/BSP Register -> 运行期按逻辑 ID 调用
```

## 🎯 中断优先级策略（当前）

| 优先级 | 中断源 | 说明 |
|:---:|------|------|
| 0 | 控制节拍相关定时器 | 控制回路核心节拍 |
| 1 | 编码器相关 EXTI | 关键传感输入 |
| 2 | USART3 | JY61P 陀螺仪实时数据流 |
| 3 | USART1/2/4 + 缺省 | 调试与通信 |

同步更新不及时，实际宏定义请以 `SYSTEM/IrqPriority.h` 为准。

## ⚙️ 构建与烧录

### ⌨️ VS Code 快捷键

| 快捷键 | 功能 |
|--------|------|
| `F7` | 编译（Build / Debug 预设） |
| `F8` | 烧录（Flash） |

### 🧪 命令行

```bash
cmake --preset Debug
cmake --build --preset Debug
```

### 🛰️ OpenOCD

- 使用配置：`OpenOCD/G3507_OpenOCD.cfg`

## 📖 文档入口

- 架构深度文档：`docs/arch-guide.md`
- 架构来源框架：OmniLayer

## ⚠️ 注意事项

- 当前仓库是 **G3507 单平台维护**，不再保留多 MCU 编译流程
- 主力维护环境是 **VS Code + CMake**

## 📮 联系

- QQ 邮箱：634591772@qq.com
