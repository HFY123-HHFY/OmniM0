#!/usr/bin/env python3
"""
OmniM0 开发环境一键安装脚本
============================
自动检测、安装、验证嵌入式开发工具链：
  - GCC ARM Embedded (arm-none-eabi-gcc)
  - CMake + Ninja
  - OpenOCD (MSPM0 烧录)

用法:
  python tools/setup_env.py              # 交互式：检测 → 安装 → 验证
  python tools/setup_env.py --check      # 仅检测，不安装（安全模式）
  python tools/setup_env.py --install    # 全自动安装，跳过确认

安全保证:
  - 已安装的工具不会被覆盖或修改
  - 所有安装通过 winget（Windows 官方包管理器），不直接操作 PATH
  - 脚本无外部依赖，仅使用 Python 标准库
"""

import argparse
import io
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

# ============================================================
# 项目根目录 & 常量
# ============================================================
PROJECT_ROOT = Path(__file__).resolve().parent.parent
VERSION = "1.0.0"

# ============================================================
# 终端颜色
# ============================================================
_ENABLE_COLOR = True


def _init_color():
    """Windows 下启用 ANSI 颜色序列 + 修复 UTF-8 输出"""
    global _ENABLE_COLOR
    if platform.system() == "Windows":
        try:
            os.system("")
        except Exception:
            _ENABLE_COLOR = False
    # 强制 stdout 使用 UTF-8，避免 emoji 导致 GBK 编码错误
    try:
        sys.stdout = io.TextIOWrapper(
            sys.stdout.buffer, encoding="utf-8", errors="replace"
        )
    except Exception:
        pass


class C:
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    CYAN = "\033[96m"
    BLUE = "\033[94m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RESET = "\033[0m"

    @staticmethod
    def g(text):
        return f"{C.GREEN}{text}{C.RESET}" if _ENABLE_COLOR else text

    @staticmethod
    def y(text):
        return f"{C.YELLOW}{text}{C.RESET}" if _ENABLE_COLOR else text

    @staticmethod
    def r(text):
        return f"{C.RED}{text}{C.RESET}" if _ENABLE_COLOR else text

    @staticmethod
    def c(text):
        return f"{C.CYAN}{text}{C.RESET}" if _ENABLE_COLOR else text

    @staticmethod
    def b(text):
        return f"{C.BOLD}{text}{C.RESET}" if _ENABLE_COLOR else text

    @staticmethod
    def dim(text):
        return f"{C.DIM}{text}{C.RESET}" if _ENABLE_COLOR else text


# ============================================================
# 工具链定义
# ============================================================
class ToolDef:
    """单个工具的定义"""

    def __init__(
        self,
        tool_id: str,
        name: str,
        exe: str,
        version_args: list,
        version_regex: str,
        min_version: tuple,
        recommended: str,
        winget_id: str,
        manual_url: str,
        manual_note: str,
        alt_exes: list = None,
        version_line: int = 0,
    ):
        self.tool_id = tool_id
        self.name = name
        self.exe = exe
        self.version_args = version_args
        self.version_regex = version_regex
        self.min_version = min_version
        self.recommended = recommended
        self.winget_id = winget_id
        self.manual_url = manual_url
        self.manual_note = manual_note
        self.alt_exes = alt_exes or []
        self.version_line = version_line  # 取输出第几行（0=首行）


# 工具链列表
TOOLS = [
    ToolDef(
        tool_id="gcc-arm",
        name="GCC ARM Embedded",
        exe="arm-none-eabi-gcc",
        version_args=["--version"],
        version_regex=r"(\d+\.\d+\.\d+)",
        min_version=(10, 0, 0),
        recommended="13.2.1",
        winget_id="Arm.GnuArmEmbeddedToolchain",
        manual_url="https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads",
        manual_note=(
            "下载 Windows .exe 安装包（推荐 13.2 Rel1），"
            "安装时务必勾选 'Add path to environment variable'"
        ),
        version_line=0,
    ),
    ToolDef(
        tool_id="cmake",
        name="CMake",
        exe="cmake",
        version_args=["--version"],
        version_regex=r"(\d+\.\d+\.\d+)",
        min_version=(3, 22, 0),
        recommended="3.22+",
        winget_id="Kitware.CMake",
        manual_url="https://cmake.org/download/",
        manual_note=(
            "下载 Windows x64 Installer，安装时选择 'Add CMake to system PATH'"
        ),
        version_line=0,
    ),
    ToolDef(
        tool_id="ninja",
        name="Ninja",
        exe="ninja",
        version_args=["--version"],
        version_regex=r"(\d+\.\d+\.\d+)",
        min_version=(1, 10, 0),
        recommended="1.13.1",
        winget_id="Ninja-build.Ninja",
        manual_url="https://github.com/ninja-build/ninja/releases",
        manual_note=(
            "下载 ninja-win.zip，将 ninja.exe 解压到 C:\\Windows 或任意 PATH 目录"
        ),
        version_line=0,
    ),
    ToolDef(
        tool_id="openocd",
        name="OpenOCD (MSPM0)",
        exe="openocd",
        version_args=["--version"],
        version_regex=r"(\d+\.\d+\.\d+)",
        min_version=(0, 11, 0),
        recommended="0.12.0",
        winget_id="xpack-dev-tools.openocd-xpack",
        manual_url="https://github.com/openocd-org/openocd/releases",
        manual_note=(
            '下载 0.12.0 Windows 版本，解压到 C:\\openocd，将 bin 目录加入系统 PATH'
        ),
        alt_exes=["openocd-xpack"],
        version_line=0,
    ),
]

# ============================================================
# VS Code 推荐插件
# ============================================================
VSCODE_EXTENSIONS = [
    {
        "id": "ms-vscode.cpptools",
        "name": "C/C++",
        "desc": "C/C++ IntelliSense、调试、代码浏览",
    },
    {
        "id": "ms-vscode.cmake-tools",
        "name": "CMake Tools",
        "desc": "CMake Presets 集成、一键构建/调试",
    },
    {
        "id": "twxs.cmake",
        "name": "CMake",
        "desc": "CMakeLists.txt 语法高亮",
    },
    {
        "id": "marus25.cortex-debug",
        "name": "Cortex-Debug",
        "desc": "ARM Cortex-M GDB 调试（配合 OpenOCD）",
    },
]


# ============================================================
# 工具检测
# ============================================================
class DetectResult:
    """单个工具的检测结果"""

    def __init__(self, tool: ToolDef):
        self.tool = tool
        self.found = False
        self.version = ""
        self.exe_path = ""
        self.actual_exe = ""
        self.version_ok = False
        self.error = ""

    @property
    def status_icon(self):
        if not self.found:
            return C.r("✗ 未找到")
        if self.version_ok:
            return C.g("✓ 已安装")
        return C.y("⚠ 版本过低")

    @property
    def version_display(self):
        if not self.found:
            return C.dim("—")
        v = self.version
        if self.version_ok:
            return C.g(v)
        else:
            return C.y(f"{v} (需要 >={'.'.join(map(str, self.tool.min_version))})")


def _parse_version(version_str: str) -> tuple:
    """解析版本字符串为元组，如 '13.2.1' -> (13, 2, 1)"""
    parts = version_str.strip().split(".")
    result = []
    for p in parts:
        try:
            result.append(int(p))
        except ValueError:
            break
    return tuple(result) if result else (0,)


def _check_version_ok(version_str: str, min_version: tuple) -> bool:
    """检查版本是否满足最低要求"""
    actual = _parse_version(version_str)
    for i, required in enumerate(min_version):
        if i >= len(actual):
            return False
        if actual[i] > required:
            return True
        if actual[i] < required:
            return False
    return True


def detect_tool(tool: ToolDef) -> DetectResult:
    """检测单个工具：查找可执行文件 + 获取版本号"""
    result = DetectResult(tool)

    exe_names = [tool.exe] + tool.alt_exes
    for exe_name in exe_names:
        exe_path = shutil.which(exe_name)
        if exe_path is None:
            continue

        try:
            proc = subprocess.run(
                [exe_name] + tool.version_args,
                capture_output=True,
                text=True,
                timeout=15,
                env={**os.environ, "LANG": "C"},
            )
            output = (proc.stdout + proc.stderr).strip()
            if not output:
                continue

            # 取指定行
            lines = output.split("\n")
            line_idx = min(tool.version_line, len(lines) - 1)
            target_line = lines[line_idx]

            match = re.search(tool.version_regex, target_line)
            if not match:
                # fallback: 搜索全部输出
                match = re.search(tool.version_regex, output)

            if match:
                result.found = True
                result.version = match.group(1)
                result.exe_path = exe_path
                result.actual_exe = exe_name
                result.version_ok = _check_version_ok(
                    result.version, tool.min_version
                )
                return result
        except FileNotFoundError:
            continue
        except subprocess.TimeoutExpired:
            result.error = "执行超时"
            continue
        except Exception as e:
            result.error = str(e)
            continue

    return result


def detect_all() -> list:
    """检测全部工具，返回 DetectResult 列表"""
    results = []
    for tool in TOOLS:
        r = detect_tool(tool)
        results.append(r)
    return results


# ============================================================
# 安装模块
# ============================================================
def _winget_available() -> bool:
    """检查 winget 是否可用"""
    return shutil.which("winget") is not None


def install_via_winget(tool: ToolDef) -> tuple:
    """
    通过 winget 安装工具。
    返回 (success: bool, message: str)
    """
    if not _winget_available():
        return False, "winget 不可用（需要 Windows 10 1809+）"

    if not tool.winget_id:
        return False, "该工具无 winget 包"

    print(f"\n  {C.c('📦')} winget install {C.b(tool.winget_id)} ...")
    print(f"  {C.dim('（下载可能较慢，请耐心等待...）')}")

    try:
        result = subprocess.run(
            [
                "winget",
                "install",
                tool.winget_id,
                "--accept-source-agreements",
                "--accept-package-agreements",
                "--disable-interactivity",
            ],
            capture_output=True,
            text=True,
            timeout=600,  # 10 分钟超时（大文件下载）
        )

        output = result.stdout + result.stderr

        # winget 可能返回非零但实际成功
        success_markers = [
            "已成功安装",
            "successfully installed",
            "已安装",
            "已修改",
            "No applicable update found",  # 已是最新
            "No installed package found",  # 需重新安装的边界情况
        ]

        is_success = result.returncode == 0 or any(
            m.lower() in output.lower() for m in success_markers
        )

        if is_success:
            return True, "安装完成"
        else:
            # 提取有用的错误信息
            error_lines = [l for l in output.split("\n") if "error" in l.lower()]
            short_error = error_lines[0][:200] if error_lines else output[-200:]
            return False, f"安装失败: {short_error}"

    except subprocess.TimeoutExpired:
        return False, "安装超时（网络较慢，请检查网络后重试）"
    except Exception as e:
        return False, f"执行错误: {e}"


# ============================================================
# VS Code 扩展推荐
# ============================================================
def write_extensions_json():
    """写入/更新 .vscode/extensions.json"""
    vscode_dir = PROJECT_ROOT / ".vscode"
    vscode_dir.mkdir(exist_ok=True)

    ext_file = vscode_dir / "extensions.json"
    recommendations = [ext["id"] for ext in VSCODE_EXTENSIONS]

    data = {"recommendations": recommendations}

    with open(ext_file, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
        f.write("\n")

    return ext_file


# ============================================================
# 显示输出
# ============================================================
def print_banner():
    """打印横幅"""
    print()
    print(C.b(C.c("  ╔══════════════════════════════════════════╗")))
    print(C.b(C.c("  ║     Omni架构 开发环境安装脚本             ║ ")))
    print(C.b(C.c("  ║     GCC ARM + CMake + Ninja + OpenOCD    ║")))
    print(C.b(C.c("  ╚══════════════════════════════════════════╝")))
    print(f"  {C.dim('版本')} {VERSION}    {C.dim('目标平台')} ARM内核")
    print()


def print_detect_report(results: list):
    """打印检测报告表格"""
    print(C.b("  📋 环境检测报告"))
    print(f"  {'─' * 62}")
    print(f"  {'工具':<24} {'状态':<14} {'版本':<16}")
    print(f"  {'─' * 62}")

    for r in results:
        status = r.status_icon
        version_display = r.version_display
        print(f"  {r.tool.name:<24} {status:<22} {version_display:<16}")
        if r.error:
            print(f"  {'':24} {C.dim(f'({r.error})')}")

    print(f"  {'─' * 62}")

    all_ok = all(r.found and r.version_ok for r in results)
    all_found = all(r.found for r in results)
    missing = [r for r in results if not r.found]
    outdated = [r for r in results if r.found and not r.version_ok]

    print()
    if all_ok:
        print(C.g("  ✅ 所有工具链已就绪，可以开始开发！"))
    else:
        if missing:
            names = ", ".join(r.tool.name for r in missing)
            print(C.r(f"  ❌ 缺少工具: {names}"))
        if outdated:
            names = ", ".join(r.tool.name for r in outdated)
            print(C.y(f"  ⚠️  版本过低: {names}"))
        if not all_found:
            absent_count = len(missing)
            print(C.dim(f"  缺失 {absent_count} 个工具，推荐安装以确保兼容性"))

    print()
    return all_ok, missing, outdated


def print_vscode_extensions():
    """打印 VS Code 推荐插件"""
    print(C.b("  🔌 VS Code 推荐插件"))
    print(f"  {'─' * 50}")
    for ext in VSCODE_EXTENSIONS:
        print(f"  {C.c(ext['id'])}")
        print(f"    {ext['desc']}")
    print()
    print(
        C.dim(
            "  已写入 .vscode/extensions.json，用 VS Code 打开工程时会自动提示安装"
        )
    )
    print()


def print_manual_install_guide(missing: list):
    """打印手动安装指南（winget 失败时的兜底方案）"""
    print()
    print(C.y("  ═══════════════════════════════════════════"))
    print(C.y("  📖 手动安装指南"))
    print(C.y("  ═══════════════════════════════════════════"))
    print()

    for r in missing:
        t = r.tool
        print(f"  {C.b(t.name)}")
        print(f"    下载: {C.c(t.manual_url)}")
        print(f"    说明: {t.manual_note}")
        print()

    print(C.dim("  安装完成后请重新打开终端，再运行本脚本验证。"))
    print()


def print_final_verify(results: list):
    """打印最终验证结果"""
    print()
    print(C.b("  🎯 最终验证"))
    print(f"  {'─' * 62}")

    for r in results:
        if r.found:
            icon = C.g("✅") if r.version_ok else C.y("⚠️")
            print(
                f"  {icon} {C.b(r.tool.name)} {C.g(r.version)}"
                + (f"  ({r.actual_exe})" if r.actual_exe != r.tool.exe else "")
            )
        else:
            print(f"  {C.r('❌')} {C.b(r.tool.name)} {C.r('未安装')}")

    print(f"  {'─' * 62}")

    all_ok = all(r.found and r.version_ok for r in results)
    if all_ok:
        print()
        print(C.g("  🎉 环境安装完成！可以开始构建工程："))
        print()
        print(f"  {C.c('  cmake --preset Debug')}")
        print(f"  {C.c('  cmake --build --preset Debug')}")
        print()
    else:
        print()
        print(C.y("  ⚠️  部分工具未就绪，请参考上方手动安装指南。"))
        print()


# ============================================================
# 主流程
# ============================================================
def confirm(prompt: str, default: bool = True) -> bool:
    """询问用户确认"""
    suffix = " [Y/n]: " if default else " [y/N]: "
    try:
        answer = input(prompt + suffix).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False

    if not answer:
        return default
    return answer in ("y", "yes", "是")


def main():
    _init_color()

    parser = argparse.ArgumentParser(
        description="OmniM0 开发环境一键安装脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python tools/setup_env.py              # 交互式：检测 → 安装 → 验证
  python tools/setup_env.py --check      # 仅检测，不安装
  python tools/setup_env.py --install    # 全自动安装（跳过确认）
        """,
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="仅检测环境状态，不执行任何安装操作",
    )
    parser.add_argument(
        "--install",
        action="store_true",
        help="全自动安装缺失的工具（跳过交互确认）",
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"OmniM0 setup_env.py v{VERSION}",
    )

    args = parser.parse_args()

    print_banner()

    # ================================================================
    # 第一阶段：检测
    # ================================================================
    print(C.b("  [1/4] 检测当前环境..."))
    results = detect_all()
    all_ok, missing, outdated = print_detect_report(results)

    # --check 模式：仅检测，不安装
    if args.check:
        if missing:
            print(
                C.dim(
                    "  提示：运行 python tools/setup_env.py 可以自动安装缺失工具。"
                )
            )
        sys.exit(0 if all_ok else 1)

    # 环境已就绪
    if all_ok and not outdated:
        print(C.g("  环境已就绪，无需安装。"))
        print_vscode_extensions()
        sys.exit(0)

    # ================================================================
    # 第二阶段：确认安装
    # ================================================================
    print(C.b("  [2/4] 准备安装..."))

    # 确定需要安装的工具
    to_install = missing + outdated

    if not args.install:
        if not to_install:
            print(C.g("  所有工具已是最新版本，无需安装。"))
            print_vscode_extensions()
            sys.exit(0)

        print()
        for r in to_install:
            if not r.found:
                print(f"    安装 {C.b(r.tool.name)}")
            else:
                print(
                    f"    升级 {C.b(r.tool.name)} "
                    f"({r.version} -> {r.tool.recommended})"
                )
        print()

        winget_ok = _winget_available()
        if not winget_ok:
            print(C.y("  ⚠️  未检测到 winget，无法自动安装。"))
            print_manual_install_guide(to_install)
            sys.exit(1)

        if not confirm("是否自动安装以上工具？"):
            print()
            print(C.dim("  已取消。可运行 'python tools/setup_env.py --check' 仅检测。"))
            print_manual_install_guide(to_install)
            sys.exit(0)
    else:
        # --install 模式：静默确认
        if not to_install:
            print(C.g("  所有工具已是最新版本，无需安装。"))
            write_extensions_json()
            print_vscode_extensions()
            sys.exit(0)

        winget_ok = _winget_available()
        if not winget_ok:
            print(C.y("  ⚠️  未检测到 winget，无法自动安装。"))
            print_manual_install_guide(to_install)
            sys.exit(1)

    # ================================================================
    # 第三阶段：安装
    # ================================================================
    print(C.b("  [3/4] 开始安装..."))

    install_ok = []
    install_fail = []

    for i, r in enumerate(to_install, 1):
        t = r.tool
        print(f"\n  [{i}/{len(to_install)}] {C.b(t.name)}")

        success, msg = install_via_winget(t)
        if success:
            print(f"  {C.g('  ✅')} {msg}")
            install_ok.append(r)
        else:
            print(f"  {C.r('  ❌')} {msg}")
            install_fail.append(r)

    # 安装完成后短暂等待（确保 PATH 环境变量更新）
    if install_ok:
        print(f"\n  {C.dim('等待系统更新环境变量...')}")
        time.sleep(2)

        # 重新检测已安装的工具
        for r in install_ok:
            new_result = detect_tool(r.tool)
            if new_result.found:
                r.found = new_result.found
                r.version = new_result.version
                r.exe_path = new_result.exe_path
                r.actual_exe = new_result.actual_exe
                r.version_ok = new_result.version_ok
                r.error = new_result.error
                print(
                    f"  {C.g('  ✅')} {r.tool.name} {C.g(r.version)} — 已就绪"
                )
            else:
                print(
                    f"  {C.y('  ⚠️')} {r.tool.name} 安装完成但未在 PATH 中找到。"
                    f" 请重新打开终端后重试。"
                )

    # 如果有安装失败的，打印手动指南
    if install_fail:
        print_manual_install_guide(install_fail)

    # ================================================================
    # 第四阶段：验证 + VS Code
    # ================================================================
    print()
    print(C.b("  [4/4] 最终验证 & VS Code 插件..."))

    # 写入 extensions.json
    ext_file = write_extensions_json()
    print(f"  {C.g('✅')} .vscode/extensions.json 已更新")

    # 最终全量检测
    final_results = detect_all()

    # 打印 VS Code 插件推荐
    print_vscode_extensions()

    # 打印最终验证
    print_final_verify(final_results)

    # 提示需要重启终端
    if install_ok:
        print(C.y("  💡 提示：如果工具未立即生效，请关闭并重新打开终端窗口。"))
        print()

    # 返回码
    final_all_ok = all(r.found and r.version_ok for r in final_results)
    sys.exit(0 if final_all_ok else 1)


if __name__ == "__main__":
    main()
