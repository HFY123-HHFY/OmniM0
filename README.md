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
- 🚌 **软件总线** — I2C/SPI 协议层与底层 GPIO 翻转分离，速率由 `BusRate.h` 集中配置
- 🎯 **单一时基前后台** — 仅 TIMG0 一个定时器中断，ISR 直接执行全部控制任务（5ms 槽 ~185µs）
- 🌀 **双 IMU 冗余** — JY61P（UART 偏航角）+ ICM42688（SPI 六轴原始数据 + 偏航积分）
- ⚡ **A4950 电机驱动** — 快衰减单极性 PWM，方向切换自动刹车死区
- 🔢 **整数 Q16.16 PID** — ISR 热路径纯整数（< 2µs），浮点 API 保持调参体验
- ⏱️ **非阻塞延时** — 基于 1ms 系统 tick，支持多通道独立计时
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
│  ├─ Control/                 # 循线控制 + 偏航角PID + 电机输出融合
│  ├─ Control_Task/            # TIMG0 ISR 调度 + TaskManager + 非阻塞延时
│  ├─ Tasks/                   # ★ 任务链：Task_1~4（KEY1 启动, KEY2 选择）
│  ├─ Detect/                  # ★ 灰度入/离线检测器（互斥状态机）
│  ├─ PID/                     # Q16.16 整数 PID + float API
│  ├─ Filter/                  # 滤波器
│  └─ My_Usart/                # 串口 printf 重定向 + 数据包解析
├─ BSP/                        # 板级设备层
│  ├─ LED/  KEY/  OLED/  Buzzer/
│  ├─ MPU6050/                 # 六轴姿态（DMP 已启用，预留）
│  ├─ ICM42688/                # ★ TDK 六轴陀螺仪（SPI2, 5MHz, ISR 驱动）
│  ├─ A4950/                   # ★ 双路 H 桥电机驱动（4 路独立 PWM, 快衰减）
│  ├─ JY61P/                   # ★ 维特智能六轴陀螺仪（UART 主动上报，USART2）
│  ├─ gray_adc/                # ★ 8 路灰度传感器（74HC4051 模拟开关）
│  ├─ TB6612/                  # 电机驱动（底层保留，已换用 A4950）
│  ├─ BMP280/                  # 可选模块（按需启用）
│  ├─ QMC5883P/                # 可选模块（按需启用）
│  └─ NRF24L01/                # 可选模块（按需启用）
├─ API/                        # 片内外设抽象层 + 协议层
│  ├─ inc/ src/                # gpio/adc/pwm/tim/usart/exti/encoder
│  ├─ API_I2C/                 # 软件 I2C 协议层
│  └─ API_SPI/                 # 软件 SPI 协议层
├─ Enroll/                     # ★ 硬件资源注册中心
├─ Core/
│  └─ MSPM0G3507/              # G3507 底层实现（sys/gpio/tim/usart/adc/pwm/exti/encoder）
├─ SYSTEM/                     # 系统层：sys/Delay/BusRate/IrqPriority
├─ Drivers/
│  └─ Drivers_M0G3507/         # TI DriverLib + CMSIS + 启动文件
├─ OpenOCD/
│  └─ G3507_OpenOCD.cfg        # 下载配置
├─ tools/
│  └─ setup_env.py             # ★ 环境一键安装脚本
├─ docs/
│  └─ arch-guide.md            # 架构深度解析
├─ .vscode/
│  ├─ extensions.json          # VS Code 推荐插件
│  ├─ tasks.json               # VS Code 任务（F7编译 / F8烧录）
│  └─ settings.json
├─ setup_env.bat               # ★ Windows 双击启动器
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
| 0 | TIMG0 | 系统时基 1ms（ISR 执行：Key_Tick + JY61P + ICM42688 + 灰度 + 方向PID + 编码器 + 任务链） |
| 2 | Encoder EXTI / USART2 | 编码器脉冲捕获 / JY61P 陀螺仪 RX |
| 3 | USART1/3/4 + 缺省 | 调试串口、MPU6050 等 |

### 🕹️ 按键映射

| 按键 | 功能 |
|:---:|------|
| KEY1 | 任务启动/急停（toggle） |
| KEY2 | 循环选择任务 1→2→3→4→1 |
| KEY3 | 保留未使用 |

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

## 🐍 一键环境安装（Python）

如果你是新电脑 / 新队友，拿到工程后**不需要手动装工具链**，直接运行：

```bash
# 方式一：双击 setup_env.bat（Windows）
setup_env.bat

# 方式二：命令行（所有平台）
python tools/setup_env.py
```

脚本会自动完成：

| 步骤 | 说明 |
|------|------|
| 🔍 环境检测 | 扫描 `arm-none-eabi-gcc`、`cmake`、`ninja`、`openocd` 是否安装，输出版本号 |
| ⚠️ 缺失提示 | 列出缺少哪些工具，给出具体版本要求 |
| 📦 一键安装 | 通过 `winget` 自动安装缺失的工具链（需确认） |
| 🔌 插件推荐 | 写入 `.vscode/extensions.json`，VS Code 打开时自动提示安装推荐插件 |
| ✅ 最终验证 | 安装完成后重新扫描，输出全部工具名称和版本号 |

**仅检测（不安装）：**

```bash
python tools/setup_env.py --check
```

**全自动安装（跳过确认）：**

```bash
python tools/setup_env.py --install
```

### 🖥️ 运行示例输出

```
  ╔══════════════════════════════════════════╗
  ║     Omni架构 开发环境安装脚本             ║
  ║     GCC ARM + CMake + Ninja + OpenOCD    ║
  ╚══════════════════════════════════════════╝

  [1/4] 检测当前环境...
  📋 环境检测报告
  ─────────────────────────────────────────────────────────────
  工具                      状态            版本
  ─────────────────────────────────────────────────────────────
  GCC ARM Embedded          ✓ 已安装        13.2.1
  CMake                     ✓ 已安装        4.2.1
  Ninja                     ✓ 已安装        1.13.1
  OpenOCD (MSPM0)           ✓ 已安装        0.12.0
  ─────────────────────────────────────────────────────────────

  ✅ 所有工具链已就绪，可以开始开发！
```

### ⚙️ 前置依赖

脚本本身只需要 **Python 3.8+**（Windows 11 已内置，或从 [python.org](https://www.python.org/downloads/) 安装）。

### 🔧 工具链版本要求

| 工具 | 最低版本 | 推荐版本 | winget 安装 |
|------|:---:|------|------|
| GCC ARM Embedded | ≥ 10.0 | 13.2.1 | `Arm.GnuArmEmbeddedToolchain` |
| CMake | ≥ 3.22 | 4.x | `Kitware.CMake` |
| Ninja | ≥ 1.10 | 1.13.x | `Ninja-build.Ninja` |
| OpenOCD | ≥ 0.11 | 0.12.0 | `xpack-dev-tools.openocd-xpack` |

### 🛡️ 安全说明

- 脚本**不会覆盖或修改**已安装的工具
- 安装通过 `winget`（Windows 官方包管理器）执行，不直接操作 PATH
- `--check` 模式（仅检测）完全不会改动系统任何东西
- 脚本不依赖任何第三方 pip 包，仅使用 Python 标准库

## 📖 文档入口

- 架构深度文档：`docs/arch-guide.md`
- 架构来源框架：OmniLayer

## ⚠️ 注意事项

- 当前仓库是 **G3507 单平台维护**，不再保留多 MCU 编译流程
- 主力维护环境是 **VS Code + CMake**
- 中断优先级实际宏定义以 `SYSTEM/IrqPriority.h` 为准

## 📮 联系

- QQ 邮箱：634591772@qq.com
