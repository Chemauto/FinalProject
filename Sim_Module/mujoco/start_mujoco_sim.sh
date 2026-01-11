#!/bin/bash
# MuJoCo 仿真启动脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "========================================"
echo "  Go2 MuJoCo 仿真启动"
echo "========================================"
echo ""

# 检查 conda 环境
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    echo "⚠️  未检测到 conda 环境"
    echo "激活 ros2_env 环境..."
    source ~/miniconda3/etc/profile.d/conda.sh
    conda activate ros2_env
fi

echo "✓ Conda 环境: $CONDA_DEFAULT_ENV"

# 检查 Python 版本
PYTHON_VERSION=$(python3 --version | awk '{print $2}')
echo "✓ Python 版本: $PYTHON_VERSION"

# 检查 ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "加载 ROS2 环境..."
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

echo "✓ ROS2 版本: $ROS_DISTRO"
echo ""

# 检查 MuJoCo
echo "检查 MuJoCo 安装..."
if python3 -c "import mujoco" 2>/dev/null; then
    echo "✓ MuJoCo 已安装"
else
    echo "❌ MuJoCo 未安装"
    echo ""
    echo "请运行安装脚本:"
    echo "  cd $SCRIPT_DIR"
    echo "  ./install_mujoco.sh"
    echo ""
    exit 1
fi

echo ""
echo "========================================"
echo "  启动 MuJoCo 仿真"
echo "========================================"
echo ""

# 启动 MuJoCo 仿真器
python3 "$SCRIPT_DIR/mujoco_simulator.py"

echo ""
echo "仿真已退出"
