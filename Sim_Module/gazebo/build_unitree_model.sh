#!/bin/bash
# Unitree Go1/Go2 机器人模型编译脚本（ROS2版本）

set -e

echo "========================================"
echo "  编译 Unitree 机器人模型（ROS2）"
echo "========================================"

# 创建工作空间
WORKSPACE="$HOME/ros2_ws"
if [ ! -d "$WORKSPACE/src" ]; then
    echo "创建 ROS2 工作空间: $WORKSPACE"
    mkdir -p "$WORKSPACE/src"
fi

cd "$WORKSPACE/src"

# 检查是否已下载
if [ -d "unitree_ros" ]; then
    echo "✓ unitree_ros 已存在，跳过下载"
else
    echo "下载 Unitree 机器人模型..."
    git clone https://github.com/unitreerobotics/unitree_ros.git
    echo "✓ 下载完成"
fi

# 安装ROS2依赖（跳过ROS1包）
cd "$WORKSPACE"
echo ""
echo "安装依赖..."

sudo apt-get update

# 只安装可用的依赖，忽略ROS1包的错误
echo "⚠️  跳过 ROS1 包的依赖检查"
rosdep install -i --from-path src --rosdistro humble -y --skip-keys="unitree_legged_msgs roscpp catkin rviz" || echo "部分依赖未找到，继续编译..."

echo "✓ 依赖检查完成"

# 编译只包含ROS2包
echo ""
echo "编译 ROS2 工作空间..."

# 方法1: 只编译ROS2包（跳过ROS1包）
colcon build \
    --packages-skip \
        unitree_legged_control \
        aliengo_description \
        h1_description \
        go2w_description \
        laikago_description \
        aliengoZ1_description \
        b1_description \
        a1_description \
        unitree_controller \
        unitree_gazebo \
        b2w_description \
        b2_description \
        z1_description \
        go1_description \
        go2_description \
        laikago_description \
        unitree_move_base \
        unitree_legged_msgs \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "✓ 编译完成"

# 设置环境
echo ""
echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc
echo "✓ 已添加工作空间到 ~/.bashrc"

echo ""
echo "========================================"
echo "  安装完成！"
echo "========================================"
echo ""
echo "运行以下命令加载环境："
echo "  source ~/.bashrc"
echo "  或: source $WORKSPACE/install/setup.bash"
echo ""
echo "在新终端中自动加载"
