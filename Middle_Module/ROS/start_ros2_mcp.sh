#!/bin/bash
# ROS2 MCP 一键启动脚本
# 类似于: dora start dora-interactive-mcp.yaml --attach
#
# 用法:
#   ./start_ros2_mcp.sh           # 默认: 2D仿真器
#   ./start_ros2_mcp.sh --sim 2d  # 2D Pygame仿真器
#   ./start_ros2_mcp.sh --sim isaac # Isaac Sim高保真仿真
#   ./start_ros2_mcp.sh --sim real # 真实机器人（需连接Go2）

set -e

# 解析命令行参数
SIM_ENV="2d"  # 默认使用2D仿真

while [[ $# -gt 0 ]]; do
    case $1 in
        --sim)
            SIM_ENV="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --sim {2d|gazebo|isaac|real}  选择仿真环境"
            echo "                         2d    - 2D Pygame仿真器（默认）"
            echo "                         gazebo - Gazebo 3D仿真（推荐）"
            echo "                         isaac - Isaac Sim高保真仿真"
            echo "                         real  - 真实机器人"
            echo "  -h, --help              显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                     # 使用2D仿真器"
            echo "  $0 --sim 2d            # 使用2D仿真器"
            echo "  $0 --sim gazebo        # 使用Gazebo 3D仿真"
            echo "  $0 --sim isaac         # 使用Isaac Sim"
            echo "  $0 --sim real          # 使用真实机器人"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 -h 或 --help 查看帮助"
            exit 1
            ;;
    esac
done

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

# 显示仿真环境选择
case $SIM_ENV in
    2d)
        print_info "仿真环境: 2D Pygame仿真器"
        ;;
    gazebo)
        print_info "仿真环境: Gazebo 3D仿真"
        print_warning "Gazebo启动可能需要15秒左右，请耐心等待..."
        ;;
    isaac)
        print_info "仿真环境: Isaac Sim高保真仿真"
        print_warning "Isaac Sim启动可能需要30秒左右，请耐心等待..."
        ;;
    real)
        print_info "仿真环境: 真实机器人"
        print_warning "确保已连接真实Go2机器人"
        ;;
    *)
        print_error "未知仿真环境: $SIM_ENV"
        print_info "有效选项: 2d, gazebo, isaac, real"
        exit 1
        ;;
esac

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

    # 优先使用 Isaac Sim 编译的 ROS2 工作空间（Python 3.11）
    ISAAC_ROS_WS="/home/robot/work/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash"

    if [ -f "$ISAAC_ROS_WS" ]; then
        source "$ISAAC_ROS_WS"
        print_success "已加载 Isaac Sim ROS2 工作空间 (Python 3.11)"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_success "已加载系统 ROS2 Humble (Python 3.10)"
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

SIM_MODULE_PATH="$SCRIPT_DIR/../../Sim_Module/ros2_2d/simulator.py"
if [ ! -f "$SIM_MODULE_PATH" ]; then
    print_error "找不到 Sim_Module/ros2_2d/simulator.py"
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

# 根据仿真环境启动不同组件
if [ "$SIM_ENV" == "isaac" ]; then
    # Isaac Sim模式：启动Isaac Sim桥接节点
    print_info "启动 Isaac Sim 桥接节点 (后台)..."

    # 检查Isaac Sim是否可用（支持两种安装方式）
    ISAAC_AVAILABLE=false
    ISAAC_PYTHON=""

    # 检查pip安装方式
    if command -v isaacsim &>/dev/null; then
        ISAAC_PYTHON=$(which python3)
        if $ISAAC_PYTHON -c "from isaacsim import SimulationApp" 2>/dev/null; then
            ISAAC_AVAILABLE=true
            print_info "检测到pip安装的Isaac Sim"
        fi
    fi

    # 检查Omniverse安装方式
    if [ "$ISAAC_AVAILABLE" = "false" ]; then
        if python3 -c "from omni.isaac.kit import SimulationApp" 2>/dev/null; then
            ISAAC_AVAILABLE=true
            ISAAC_PYTHON=$(which python3)
            print_info "检测到Omniverse安装的Isaac Sim"
        fi
    fi

    if [ "$ISAAC_AVAILABLE" = "false" ]; then
        print_error "Isaac Sim不可用"
        print_info "请确保Isaac Sim已安装并激活正确的conda环境"
        print_info "  - pip安装: conda activate env_isaaclab"
        print_info "  - Omniverse安装: source /path/to/isaac-sim/setup.sh"
        exit 1
    fi

    # 使用正确的Python运行Isaac Sim桥接节点
    $ISAAC_PYTHON isaac_sim_bridge.py > /tmp/isaac_sim_bridge.log 2>&1 &
    BRIDGE_PID=$!
    echo $BRIDGE_PID > /tmp/isaac_sim_bridge.pid

    print_success "Isaac Sim 桥接节点已启动 (PID: $BRIDGE_PID)"
    print_info "使用Python: $ISAAC_PYTHON"
    print_info "日志文件: /tmp/isaac_sim_bridge.log"
    print_info "等待Isaac Sim初始化..."

    # Isaac Sim需要更长启动时间
    sleep 5

    # 检查桥接节点是否正常运行
    if ! ps -p $BRIDGE_PID > /dev/null 2>&1; then
        print_error "Isaac Sim 桥接节点启动失败！"
        print_info "查看日志: cat /tmp/isaac_sim_bridge.log"
        exit 1
    fi

    print_success "Isaac Sim 桥接节点运行正常"

elif [ "$SIM_ENV" == "gazebo" ]; then
    # Gazebo 3D仿真模式：启动Gazebo和机器人控制器
    print_info "启动 Gazebo 3D 仿真环境 (后台)..."
    print_info "这可能需要一些时间，请耐心等待 Gazebo 窗口弹出..."

    # 使用 gazebo 命令启动，并手动加载 ROS2 插件
    print_info "使用 gazebo 命令启动（手动指定 ROS2 插件）..."

    # 选择 world 文件（使用简单的 empty.world，插件通过命令行参数加载）
    GAZEBO_WORLD="/opt/ros/humble/share/gazebo_ros/worlds/empty.world"

    # 启动 Gazebo，手动指定要加载的 ROS2 插件
    # -slibgazebo_ros_init.so: ROS2 初始化插件
    # -slibgazebo_ros_factory.so: 提供 /spawn_entity 服务
    # -slibgazebo_ros_force_system.so: 力系统插件
    # -slibgazebo_ros_state.so: 状态发布插件
    stdbuf -oL -eL gazebo "$GAZEBO_WORLD" \
        -slibgazebo_ros_init.so \
        -slibgazebo_ros_factory.so \
        -slibgazebo_ros_force_system.so \
        -slibgazebo_ros_state.so \
        > /tmp/gazebo_sim.log 2>&1 &

    GAZEBO_PID=$!

    print_success "Gazebo 仿真环境已启动 (PID: $GAZEBO_PID)"
    print_info "日志文件: /tmp/gazebo_sim.log"

    # 等待 Gazebo 完全加载（需要更长时间）
    print_info "等待 Gazebo 初始化（约15秒）..."
    for i in {1..15}; do
        sleep 1
        echo -n "."
        # 检查进程是否还在运行
        if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
            echo ""
            print_error "Gazebo 进程意外退出！"
            print_info "查看日志: cat /tmp/gazebo_sim.log"
            exit 1
        fi
    done
    echo ""

    # 检查 Gazebo 是否正常运行
    if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
        print_error "Gazebo 启动失败！"
        print_info "查看日志: cat /tmp/gazebo_sim.log"
        print_info ""
        print_info "可能的原因："
        print_info "  1. Gazebo 需要图形界面（确保 $DISPLAY 已设置）"
        print_info "  2. 显卡驱动问题"
        print_info "  3. 可以尝试运行: ./start_gazebo_simple.sh"
        exit 1
    fi

    print_success "Gazebo 运行正常"

    # 检查 /spawn_entity 服务是否可用
    print_info "检查 Gazebo ROS2 服务..."
    sleep 2
    if ! ros2 service list 2>/dev/null | grep -q "spawn_entity"; then
        print_warning "Gazebo ROS2 服务未找到"
        print_info "尝试手动加载 Gazebo 插件..."
        print_info "这可能需要几秒钟..."
        sleep 5

        # 再次检查
        if ! ros2 service list 2>/dev/null | grep -q "spawn_entity"; then
            print_error "无法连接到 Gazebo ROS2 服务"
            print_info ""
            print_info "建议："
            print_info "  1. 确保已安装 gazebo_ros_pkgs"
            print_info "  2. 尝试运行: ros2 run gazebo_ros gazebo"
            print_info "  3. 查看日志: cat /tmp/gazebo_sim.log"
            exit 1
        fi
    fi

    print_success "Gazebo ROS2 服务已就绪"

    # 启动 robot_state_publisher（发布 robot_description 话题）
    print_info "启动 robot_state_publisher..."
    # Use system Python (3.10) for ROS2 commands
    PATH="/usr/bin:/bin:$PATH" ros2 run robot_state_publisher robot_state_publisher --ros-args \
        -p robot_description:="$(xacro /home/robot/ros2_ws/src/go2_gazebo_description/urdf/go2.urdf.xacro)" \
        > /tmp/rsp.log 2>&1 &
    RSP_PID=$!

    sleep 3

    # Spawn 机器人到 Gazebo
    print_info "Spawn Go2 机器人到场景中..."
    # Use system Python (3.10) for ROS2 commands
    PATH="/usr/bin:/bin:$PATH" ros2 run gazebo_ros spawn_entity.py \
        -entity go2 \
        -topic /robot_description \
        -x 0.0 \
        -y 0.0 \
        -z 0.3 \
        > /tmp/spawn.log 2>&1 &
    SPAWN_PID=$!

    sleep 5

    # 检查 spawn 是否成功
    if ! ps -p $SPAWN_PID > /dev/null 2>&1; then
        print_warning "Spawn 进程已退出，检查日志..."
        if grep -q "Successfully spawned entity" /tmp/spawn.log 2>/dev/null; then
            print_success "机器人 Spawn 成功！"
        else
            print_error "机器人 Spawn 失败"
            print_info "查看日志: cat /tmp/spawn.log"
        fi
    else
        print_success "机器人 Spawn 命令已发送"
    fi

    print_info "等待机器人加载完成..."
    sleep 2

    # 启动 Gazebo 机器人控制器
    print_info "启动 Gazebo Robot Controller (后台)..."

    # Use ROS2's Python (3.10) instead of conda's Python
    /usr/bin/python3 gazebo_robot_controller.py > /tmp/gazebo_controller.log 2>&1 &
    CONTROLLER_PID=$!
    
    # 启动 Mecanum 轮运动学控制器
    print_info "启动 Mecanum Control Node (后台)..."
    python3 mechControl.py > /tmp/mech_control.log 2>&1 &
    MECH_PID=$!

    print_success "Gazebo Robot Controller 已启动 (PID: $CONTROLLER_PID)"
    print_success "Mecanum Control Node 已启动 (PID: $MECH_PID)"
    print_info "日志文件: /tmp/gazebo_controller.log and /tmp/mech_control.log"
    
    # 等待控制器初始化
    sleep_and_check() {
        if ! ps -p $1 > /dev/null; then
            print_error "$2 启动失败"
            print_info "查看日志: cat $3"
            # Optional: kill other processes before exiting
            return 1
        fi
        return 0
    }
    sleep 2
    sleep_and_check $CONTROLLER_PID "Gazebo Robot Controller" "/tmp/gazebo_controller.log" || exit 1
    sleep_and_check $MECH_PID "Mecanum Control Node" "/tmp/mech_control.log" || exit 1

    print_success "所有控制器运行正常"

elif [ "$SIM_ENV" == "2d" ]; then
    # ... (rest of the script)

    # 2D仿真模式：启动2D仿真器和控制器
    print_info "启动 ROS2 Simulator (后台)..."

    python3 "$SIM_MODULE_PATH" > /tmp/ros2_simulator.log 2>&1 &
    SIMULATOR_PID=$!

    print_success "ROS2 Simulator 已启动 (PID: $SIMULATOR_PID)"
    print_info "日志文件: /tmp/ros2_simulator.log"

    # 等待 simulator 初始化
    sleep 1

    print_info "启动 ROS2 Robot Controller (后台)..."

    python3 ros2_robot_controller.py --robot Sim_2D > /tmp/ros2_controller.log 2>&1 &
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

elif [ "$SIM_ENV" == "real" ]; then
    # 真实机器人模式：只启动控制器（不启动仿真器）
    print_info "真实机器人模式：只启动 ROS2 Robot Controller..."

    python3 ros2_robot_controller.py --robot Go2_Quadruped > /tmp/ros2_controller.log 2>&1 &
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
fi

# 启动交互式 MCP（前台）
print_header "启动交互式 MCP (前台)"

print_info "运行 ros2_interactive_mcp.py..."
print_info ""
print_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
print_info "  ROS2 MCP 系统已就绪！"
print_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
print_info ""

# 根据仿真环境显示不同信息
if [ "$SIM_ENV" == "2d" ]; then
    print_info "📺 已启动组件："
    print_info "  ✓ ROS2 Simulator (2D可视化仿真窗口)"
    print_info "  ✓ ROS2 Robot Controller (机器人控制)"
    print_info "  ✓ 交互式 MCP (命令行界面)"
    print_info ""
    print_info "💡 使用提示:"
    print_info "  - 2D仿真窗口会显示机器人位置和运动"
elif [ "$SIM_ENV" == "gazebo" ]; then
    print_info "📺 已启动组件："
    print_info "  ✓ Gazebo (3D物理仿真环境)"
    print_info "  ✓ Unitree 四足机器人"
    print_info "  ✓ Gazebo Robot Controller (机器人控制)"
    print_info "  ✓ 交互式 MCP (命令行界面)"
    print_info ""
    print_info "💡 使用提示:"
    print_info "  - Gazebo提供了真实的物理仿真"
    print_info "  - 支持四足机器人动力学仿真"
    print_info "  - 适合 Sim2Real 算法开发"
elif [ "$SIM_ENV" == "isaac" ]; then
    print_info "📺 已启动组件："
    print_info "  ✓ Isaac Sim (高保真物理仿真)"
    print_info "  ✓ ROS2-Isaac Sim Bridge (桥接节点)"
    print_info "  ✓ 交互式 MCP (命令行界面)"
    print_info ""
    print_info "💡 使用提示:"
    print_info "  - Isaac Sim提供了真实的物理仿真"
    print_info "  - 支持四足机器人动力学仿真"
elif [ "$SIM_ENV" == "real" ]; then
    print_info "📺 已启动组件："
    print_info "  ✓ 真实Go2机器人"
    print_info "  ✓ ROS2 Robot Controller (机器人控制)"
    print_info "  ✓ 交互式 MCP (命令行界面)"
    print_info ""
    print_info "💡 使用提示:"
    print_info "  - 确保Go2机器人已连接并开机"
fi

print_info "  - 在此终端输入自然语言指令，例如："
print_info '    "前进1米"'
print_info '    "先左转90度，再往前走1米"'
print_info '    "抓取杯子"'
print_info "  - 输入 'q' 或 'quit' 退出"
print_info "  - 按 Ctrl+C 退出系统"
print_info ""
print_warning "⚠️  重要: 请使用 Ctrl+C 退出，而不是 Ctrl+Z。"
print_warning "   Ctrl+Z 仅会挂起进程，不会触发自动清理，可能导致进程残留。"
print_info ""
print_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
print_info ""

# 设置退出时的清理函数
cleanup() {
    print_info ""
    print_header "正在清理后台进程，请稍候..."

    # 清理2D仿真器
    if [ -n "$SIMULATOR_PID" ]; then
        print_info "终止 ROS2 Simulator (PID: $SIMULATOR_PID)..."
        kill $SIMULATOR_PID 2>/dev/null || true
    fi

    # 清理Gazebo
    if [ -n "$GAZEBO_PID" ]; then
        print_info "终止 Gazebo (PID: $GAZEBO_PID)..."
        kill $GAZEBO_PID 2>/dev/null || true
    fi

    # 清理Isaac Sim桥接节点
    if [ -n "$BRIDGE_PID" ]; then
        print_info "终止 Isaac Sim 桥接节点 (PID: $BRIDGE_PID)..."
        kill $BRIDGE_PID 2>/dev/null || true
    fi

    # 清理ROS2控制器
    if [ -n "$CONTROLLER_PID" ]; then
        print_info "终止 ROS2 Robot Controller (PID: $CONTROLLER_PID)..."
        kill $CONTROLLER_PID 2>/dev/null || true
    fi

    # 清理 robot_state_publisher
    if [ -n "$RSP_PID" ]; then
        print_info "终止 robot_state_publisher (PID: $RSP_PID)..."
        kill $RSP_PID 2>/dev/null || true
    fi

    # 额外清理：查找并终止任何残留的相关进程
    # 使用 pkill 确保所有相关进程都被终止
    print_info "执行最后的清理检查..."
    pkill -f "simulator.py" 2>/dev/null || true
    pkill -f "ros2_simulator.py" 2>/dev/null || true
    pkill -f "isaac_sim_bridge.py" 2>/dev/null || true
    pkill -f "ros2_robot_controller.py" 2>/dev/null || true
    pkill -f "gazebo_robot_controller.py" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    
    # 强制终止所有与Gazebo相关的进程
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
    pkill -9 -f "gazebo" 2>/dev/null || true
    
    # 等待一小段时间，确保进程有时间退出
    sleep 1

    print_success "所有后台进程已清理完毕"
    print_info "再见！"
    exit 0
}

# 捕获退出信号
# INT: Ctrl+C
# TERM: kill 命令
# EXIT: 脚本正常退出
# 当捕获到这些信号时，执行 cleanup 函数
trap cleanup INT TERM EXIT

# 启动交互式 MCP（阻塞）
if [ "$SIM_ENV" == "gazebo" ]; then
    print_info "启动 Gazebo 专用的交互式 MCP..."
    python3 ros2_interactive_mcp_gazebo.py
else
    print_info "启动标准的交互式 MCP..."
    python3 ros2_interactive_mcp.py
fi

# 脚本执行到这里时，通常是因为 python3 进程退出了
# trap on EXIT 会自动调用 cleanup 函数
