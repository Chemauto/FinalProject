#!/bin/bash
# Gazebo + ROS2 集成安装脚本
# 方案1: 优先使用 Gazebo Fortress (新版)
# 方案2: 回退到 Gazebo 11 (经典版，与 ROS2 Humble 兼容)

set -e

echo "========================================"
echo "  安装 Gazebo + ROS2-Gazebo"
echo "========================================"

# 步骤 1: 安装 Gazebo
echo ""
echo "[1/3] 安装 Gazebo..."

# 添加 Gazebo 软件源
if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
    sudo wget https://packages.osrfoundation.org/gazebo.asc -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
fi

# 更新软件源
sudo apt-get update

# 尝试安装新版 Gazebo (Fortress)
echo "尝试安装 Gazebo Fortress..."
if sudo apt-get install gz-fortress -y 2>/dev/null; then
    echo "✓ Gazebo Fortress 安装成功"
    GAZEBO_VERSION="fortress"
else
    echo "⚠️  Gazebo Fortress 不可用，安装经典版 Gazebo 11..."
    sudo apt-get install gazebo11 libgazebo11-dev -y
    echo "✓ Gazebo 11 安装成功"
    GAZEBO_VERSION="11"
fi

# 步骤 2: 安装 ROS2-Gazebo 集成插件
echo ""
echo "[2/3] 安装 ROS2-Gazebo 集成插件..."

# ROS2 Humble 的 Gazebo 集成包
sudo apt-get install ros-humble-gazebo-ros-pkgs -y
sudo apt-get install ros-humble-ros-gz -y 2>/dev/null || echo "⚠️  ros-gz 不可用（仅新版 Gazebo 支持）"
sudo apt-get install ros-humble-gazebo-ros2-control -y 2>/dev/null || echo "⚠️  gazebo-ros2-control 不可用"

echo "✓ ROS2-Gazebo 插件安装完成"

# 步骤 3: 安装其他依赖
echo ""
echo "[3/3] 安装其他依赖..."

sudo apt-get install \
    ros-humble-ros2-control \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-diff-drive-controller \
    -y

echo "✓ 依赖安装完成"

# 验证安装
echo ""
echo "========================================"
echo "  验证安装"
echo "========================================"

echo "Gazebo 版本: $GAZEBO_VERSION"

if [ "$GAZEBO_VERSION" == "fortress" ]; then
    gz sim --version || echo "Gazebo Fortress 已安装"
else
    gazebo --version || echo "Gazebo 11 已安装"
fi

# 检查 ROS2 Gazebo 包
echo ""
echo "ROS2 Gazebo 包:"
dpkg -l | grep gazebo || echo "无"

echo ""
echo "✓ 安装完成！"
echo ""
echo "下一步：编译 Unitree 机器人模型"
echo "运行: ./build_unitree_model.sh"
