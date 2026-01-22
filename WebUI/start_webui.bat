@echo off
REM WebUI 一键启动脚本 (Windows)
chcp 65001 >nul

setlocal enabledelayedexpansion

echo [INFO] Robot WebUI 启动脚本
echo ========================================

REM 检查 .env 文件
if not exist "..\.env" (
    echo [WARNING] 未找到 .env 文件
    echo [INFO] 后端将尝试从环境变量加载 API Key
) else (
    echo [SUCCESS] 找到 .env 文件
)

REM 检查依赖
echo [INFO] 检查依赖...

where python >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] 未找到 python
    exit /b 1
)

where npm >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] 未找到 npm
    exit /b 1
)

echo [SUCCESS] 依赖检查完成

REM 安装后端依赖
echo [INFO] 检查后端依赖...
cd /d "%~dp0backend"

if not exist "venv" (
    echo [INFO] 创建 Python 虚拟环境...
    python -m venv venv
)

call venv\Scripts\activate.bat
pip install -q -r requirements.txt
echo [SUCCESS] 后端依赖就绪

REM 安装前端依赖
echo [INFO] 检查前端依赖...
cd /d "%~dp0frontend"

if not exist "node_modules" (
    echo [INFO] 安装前端依赖（首次运行可能需要几分钟）...
    call npm install --silent
)

echo [SUCCESS] 前端依赖就绪

REM 构建前端
echo [INFO] 构建前端...
call npm run build
echo [SUCCESS] 前端构建完成

REM 启动后端服务器
cd /d "%~dp0backend"
echo [INFO] 启动后端服务器...
echo.
echo [INFO] 访问地址: http://localhost:5000
echo [INFO] 按 Ctrl+C 停止服务器
echo ========================================
echo.

call venv\Scripts\activate.bat
python app.py
