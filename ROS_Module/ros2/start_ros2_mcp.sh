#!/bin/bash
# ROS2 MCP 一键启动脚本
# 类似于: dora start dora-interactive-mcp.yaml --attach

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_header() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

print_header "ROS2 MCP 交互式系统 - 启动中"

# 检查 conda 环境
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    print_warning "未检测到 conda 环境"
    print_info "尝试自动激活 ros2_env 环境..."

    if conda activate ros2_env 2>/dev/null; then
        print_success "已激活 ros2_env 环境"
    else
        print_error "无法激活 ros2_env 环境"
        echo ""
        echo "请先创建并激活 conda 环境:"
        echo "  conda create -n ros2_env python=3.10 -y"
        echo "  conda activate ros2_env"
        echo "  pip install mcp openai"
        exit 1
    fi
else
    print_success "Conda 环境: $CONDA_DEFAULT_ENV"
fi

# 检查 ROS2
if [ -z "$ROS_DISTRO" ]; then
    print_warning "ROS2 环境未加载"
    print_info "尝试加载 ROS2 环境..."

    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_success "已加载 ROS2 Humble 环境"
    else
        print_error "找不到 ROS2 Humble 安装"
        echo ""
        echo "请先安装 ROS2 Humble，参考 ROS2_QUICKSTART.md"
        exit 1
    fi
else
    print_success "ROS2 版本: $ROS_DISTRO"
fi

# 检查 Python 版本
PYTHON_VERSION=$(python3 --version | awk '{print $2}')
print_info "Python 版本: $PYTHON_VERSION"

if [[ ! "$PYTHON_VERSION" =~ ^3\.10 ]]; then
    print_warning "ROS2 Humble 推荐 Python 3.10，当前版本: $PYTHON_VERSION"
fi

# 检查必要文件
print_info "检查必要文件..."

if [ ! -f "ros2_interactive_mcp.py" ]; then
    print_error "找不到 ros2_interactive_mcp.py"
    exit 1
fi

if [ ! -f "ros2_robot_controller.py" ]; then
    print_error "找不到 ros2_robot_controller.py"
    exit 1
fi

if [ ! -f "ros2_simulator.py" ]; then
    print_error "找不到 ros2_simulator.py"
    exit 1
fi

PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ ! -f "$PROJECT_ROOT/.env" ]; then
    print_warning ".env 文件不存在"
    print_info "将在项目根目录创建 .env 文件或使用环境变量"
fi

print_success "所有必要文件检查完成"

# 检查 pygame（仿真器需要）
print_info "检查 pygame..."
if ! python3 -c "import pygame" 2>/dev/null; then
    print_warning "pygame 未安装，仿真器可能无法运行"
    print_info "安装命令: pip install pygame"
fi

# 检查是否有进程在运行
print_info "检查是否有 ROS2 节点在运行..."

if pgrep -f "ros2_robot_controller.py" > /dev/null; then
    print_warning "检测到 ros2_robot_controller.py 已在运行"
    read -p "是否终止现有进程并重新启动? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "终止现有进程..."
        pkill -f "ros2_robot_controller.py" || true
        sleep 1
    else
        print_info "继续使用现有进程"
    fi
fi

print_header "启动 ROS2 MCP 系统"

# 启动 ROS2 Simulator（后台）
print_info "启动 ROS2 Simulator (后台)..."

python3 ros2_simulator.py > /tmp/ros2_simulator.log 2>&1 &
SIMULATOR_PID=$!

print_success "ROS2 Simulator 已启动 (PID: $SIMULATOR_PID)"
print_info "日志文件: /tmp/ros2_simulator.log"

# 等待 simulator 初始化
sleep 1

# 启动 ROS2 Robot Controller（后台）
print_info "启动 ROS2 Robot Controller (后台)..."

# 使用 subprocess 在后台启动 controller
python3 ros2_robot_controller.py > /tmp/ros2_controller.log 2>&1 &
CONTROLLER_PID=$!

print_success "ROS2 Robot Controller 已启动 (PID: $CONTROLLER_PID)"
print_info "日志文件: /tmp/ros2_controller.log"

# 等待 controller 初始化
print_info "等待 ROS2 Robot Controller 初始化..."
sleep 2

# 检查 controller 是否成功启动
if ps -p $CONTROLLER_PID > /dev/null; then
    print_success "ROS2 Robot Controller 运行正常"
else
    print_error "ROS2 Robot Controller 启动失败"
    print_info "查看日志: cat /tmp/ros2_controller.log"
    exit 1
fi

# 启动交互式 MCP（前台）
print_header "启动交互式 MCP (前台)"

print_info "运行 ros2_interactive_mcp.py..."
print_info ""
print_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
print_info "  ROS2 MCP 系统已就绪！"
print_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
print_info ""
print_info "📺 已启动组件："
print_info "  ✓ ROS2 Simulator (可视化仿真窗口)"
print_info "  ✓ ROS2 Robot Controller (机器人控制)"
print_info "  ✓ 交互式 MCP (命令行界面)"
print_info ""
print_info "💡 使用提示:"
print_info "  - 仿真窗口会显示机器人位置和运动"
print_info "  - 在此终端输入自然语言指令，例如："
print_info '    "前进1米"'
print_info '    "先左转90度，再往前走1米"'
print_info '    "抓取杯子"'
print_info "  - 输入 'q' 或 'quit' 退出"
print_info "  - 按 Ctrl+C 或关闭仿真窗口退出"
print_info ""
print_info "⚠️  退出时会自动清理所有后台进程"
print_info ""
print_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
print_info ""

# 设置退出时的清理函数
cleanup() {
    print_info ""
    print_info "清理后台进程..."

    if ps -p $SIMULATOR_PID > /dev/null 2>&1; then
        print_info "终止 ROS2 Simulator (PID: $SIMULATOR_PID)..."
        kill $SIMULATOR_PID 2>/dev/null || true
        wait $SIMULATOR_PID 2>/dev/null || true
    fi

    if ps -p $CONTROLLER_PID > /dev/null 2>&1; then
        print_info "终止 ROS2 Robot Controller (PID: $CONTROLLER_PID)..."
        kill $CONTROLLER_PID 2>/dev/null || true
        wait $CONTROLLER_PID 2>/dev/null || true
    fi

    print_success "已清理"
    print_info "再见！"
}

# 捕获退出信号
trap cleanup EXIT INT TERM

# 启动交互式 MCP（阻塞）
python3 ros2_interactive_mcp.py

# 退出时会自动调用 cleanup
