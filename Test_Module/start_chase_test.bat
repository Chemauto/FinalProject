@echo off
REM 追击功能测试启动脚本 (Windows)

setlocal enabledelayedexpansion
chcp 65001 >nul

echo ========================================
echo 追击功能测试 - 启动脚本
echo ========================================
echo.

REM 检查 Python
where python >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未找到 Python
    exit /b 1
)

REM 进入项目目录
cd /d "%~dp0.."

echo [信息] 项目目录: %CD%
echo.

REM 检查 .env
if not exist ".env" (
    echo [警告] 未找到 .env 文件
    echo [信息] 请确保已设置 Test_API_KEY
    echo.
) else (
    echo [成功] 找到 .env 文件
    echo.
)

echo ========================================
echo 请选择启动模式:
echo ========================================
echo.
echo 1. 仅启动仿真器
echo 2. 仅启动测试程序
echo 3. 同时启动（两个窗口）
echo 4. 退出
echo.

set /p choice="请输入选择 (1-4): "

if "%choice%"=="1" goto sim
if "%choice%"=="2" goto test
if "%choice%"=="3" goto both
if "%choice%"=="4" goto end

echo [错误] 无效选择
exit /b 1

:sim
echo.
echo [启动] 仿真器...
echo.
start "增强仿真器" python Test_Module/enhanced_simulator.py
goto end

:test
echo.
echo [启动] 测试程序...
echo.
start "追击测试" python Test_Module/test_chase.py
goto end

:both
echo.
echo [启动] 仿真器和测试程序...
echo.
start "增强仿真器" python Test_Module/enhanced_simulator.py
timeout /t 2 >nul
start "追击测试" python Test_Module/test_chase.py
echo.
echo [提示] 两个窗口已打开
echo [提示] 先在仿真器中按 'R' 生成敌人
echo [提示] 然后在测试程序中输入追击命令
goto end

:end
echo.
echo ========================================
echo 按任意键退出...
pause >nul
