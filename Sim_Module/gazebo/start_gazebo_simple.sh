#!/bin/bash
# 简化的 Gazebo 启动脚本

# 激活环境
source /opt/ros/humble/setup.bash
source /home/robot/ros2_ws/install/setup.bash

# 检查 go2_gazebo_description 包
if ! ros2 pkg list | grep -q "go2_gazebo_description"; then
    echo "❌ go2_gazebo_description 包未找到"
    echo "请先运行: cd /home/robot/ros2_ws && colcon build --packages-select go2_gazebo_description"
    exit 1
fi

echo "🚀 启动 Gazebo + Go2 机器人..."

# 方法：直接启动 Gazebo，然后使用 ROS2 spawn 机器人
# 启动 Gazebo（空世界）
gazebo --verbose &
GAZEBO_PID=$!

echo "✓ Gazebo 启动 (PID: $GAZEBO_PID)"
echo "⏳ 等待 Gazebo 初始化（10秒）..."
sleep 10

# Spawn Go2 机器人
echo "🤖 Spawn Go2 机器人..."
ros2 run gazebo_ros spawn_entity.py \
    -entity go2 \
    -topic /robot_description \
    -x 0.0 \
    -y 0.0 \
    -z 0.3 \
    -Y 0.0 &

# 启动 robot_state_publisher
echo "📤 启动 robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args \
    -p robot_description:="$(xacro /home/robot/ros2_ws/src/go2_gazebo_description/urdf/go2.urdf.xacro)" &

echo "✓ Go2 机器人已 spawn 到场景中"
echo ""
echo "💡 提示:"
echo "  - Gazebo 窗口应该已经打开"
echo "  - 你应该能看到 Go2 机器人"
echo "  - 按 Ctrl+C 退出"
echo ""

# 等待 Gazebo 进程
wait $GAZEBO_PID
