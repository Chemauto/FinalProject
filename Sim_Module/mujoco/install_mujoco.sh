#!/bin/bash
# MuJoCo 安装脚本
# 适用于 Ubuntu 22.04/24.04 + Python 3.10+

set -e

echo "========================================"
echo "  安装 MuJoCo 物理仿真器"
echo "========================================"

# 检查 Python 版本
PYTHON_VERSION=$(python3 --version | awk '{print $2}')
echo "Python 版本: $PYTHON_VERSION"

# 激活 conda 环境（如果存在）
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "Conda 环境: $CONDA_DEFAULT_ENV"
else
    echo "⚠️  未检测到 conda 环境"
    echo "建议使用: conda activate ros2_env"
fi

echo ""
echo "[1/3] 安装 MuJoCo Python 包..."

# 升级 pip
pip install --upgrade pip

# 安装 MuJoCo
pip install mujoco

echo "✓ MuJoCo 安装完成"

echo ""
echo "[2/3] 验证安装..."

python3 -c "import mujoco; print(f'MuJoCo 版本: {mujoco.__version__}')" 2>/dev/null || echo "⚠️  无法获取 MuJoCo 版本"

echo ""
echo "[3/3] 测试导入..."

if python3 -c "import mujoco; import mujoco.viewer; print('✓ MuJoCo 和 viewer 模块导入成功')"; then
    echo ""
    echo "========================================"
    echo "  安装完成！"
    echo "========================================"
    echo ""
    echo "MuJoCo 特性:"
    echo "  - 高性能物理仿真"
    echo "  - 准确的接触模型"
    echo "  - 内置可视化"
    echo "  - 支持强化学习"
    echo "  - 比 Gazebo 更轻量"
    echo ""
    echo "测试安装:"
    echo "  python3 -c 'import mujoco; print(mujoco.MjSpec())'"
    echo ""
else
    echo ""
    echo "❌ 导入失败，尝试安装依赖..."

    # 安装可能缺失的依赖
    pip install \
        glfw \
        PyOpenGL \
        PyOpenGL-accelerate \
        numpy \
        pillow \
        patchelf

    echo "✓ 依赖安装完成，请重新测试"
fi
