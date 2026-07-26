@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo.
echo   ╔══════════════════════════════════════════╗
echo   ║     Omni架构 开发环境安装脚本             ║
echo   ╚══════════════════════════════════════════╝
echo.

:: 检查 Python 是否可用
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo   [错误] 未检测到 Python！
    echo.
    echo   请先安装 Python 3.8+ 后再运行此脚本：
    echo   https://www.python.org/downloads/
    echo.
    echo   安装时务必勾选 "Add Python to PATH"
    echo.
    pause
    exit /b 1
)

:: 运行检测脚本（参数透传，UTF-8 环境）
set PYTHONIOENCODING=utf-8
python "%~dp0tools\setup_env.py" %*

:: 保持窗口不关闭
echo.
echo   按任意键关闭窗口...
pause >nul
