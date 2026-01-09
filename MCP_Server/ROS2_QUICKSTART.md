# ROS2 快速入门指南

本指南将帮助你快速开始使用ROS2适配器控制真实机器人。

## 📋 前置要求

- Ubuntu 20.04/22.04 (推荐) 或 WSL2 + Ubuntu
- Python 3.8+
- 管理员权限（用于安装软件）

## 🚀 5分钟快速安装

### 1. 安装ROS2 Humble

```bash
# 设置语言环境
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS2软件源
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2
sudo apt update
sudo apt install ros-humble-desktop -y

# 安装开发工具
sudo apt install python3-colcon-common-extensions -y
```

### 2. 配置环境

```bash
# 加载ROS2环境
source /opt/ros/humble/setup.bash

# 添加到.bashrc自动加载
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 验证安装

```bash
# 检查ROS2版本
printenv ROS_DISTRO  # 应该输出: humble

# 检查Python包
python3 -c "import rclpy; print('rclpy version:', rclpy.__version__)"
```

## 🤖 运行示例

### 1. 启动ROS2控制器

```bash
cd <Project_Path>/MCP_Server
python3 examples/ros2_robot_controller.py
```

你应该看到：
```
[ROS2] Robot Controller Initialized
[ROS2] Listening to topic: /robot_command
[ROS2] Publishing to topic: /cmd_vel

[ROS2] Controller is running. Press Ctrl+C to exit...
```

### 2. 发送测试命令

在另一个终端：

```bash
# 方法1: 使用MCP服务器
cd <Project_Path>/MCP_Server
python3 mcp_robot_server.py --adapter ros2

# 方法2: 手动发布测试命令
ros2 topic pub /robot_command std_msgs/String "{data: '{\"action\": \"navigate\", \"parameters\": {\"direction\": \"front\", \"distance\": \"1m\"}}'}"
```

## 📊 监控工具

### 查看话题

```bash
# 列出所有话题
ros2 topic list

# 查看命令话题
ros2 topic echo /robot_command

# 查看速度命令
ros2 topic echo /cmd_vel

# 查看话题信息
ros2 topic info /robot_command
```

### 可视化节点图

```bash
# 安装rqt_graph（如果未安装）
sudo apt install ros-humble-rqt-graph -y

# 运行节点图可视化
rqt_graph
```

### 监控节点

```bash
# 列出所有节点
ros2 node list

# 查看节点信息
ros2 node info /ros2_robot_controller
```

## 🔧 常见问题解决

### 问题1: rclpy未找到

**解决方案**:
```bash
# 确保已加载ROS2环境
source /opt/ros/humble/setup.bash

# 如果仍然失败，重新安装rclpy
sudo apt install python3-rclpy -y
```

### 问题2: 节点无法发现其他节点

**解决方案**:
```bash
# 设置ROS域ID
export ROS_DOMAIN_ID=0

# 添加到.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### 问题3: 权限错误

**解决方案**:
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 重新登录或运行
newgrp dialout
```

## 📚 下一步

- 阅读[完整ROS2教程](https://docs.ros.org/en/humble/Tutorials.html)
- 了解[ROS2概念](https://docs.ros.org/en/humble/Concepts/Basic.html)
- 探索[MCP_Server/README.md](README.md)获取更多高级用法

## 🆚 ROS1 vs ROS2

| 特性 | ROS1 | ROS2 |
|------|------|------|
| 安装 | 需要roscore | 无需master |
| Python API | rospy | rclpy |
| 安全性 | 无 | 内置加密 |
| 实时性 | 一般 | 更好（DDS） |
| 平台 | 主要Linux | 全平台 |

## 📞 获取帮助

- ROS2官方论坛: https://answers.ros.org/
- ROS2 Discord: https://discord.gg/ros2
- 本项目Issue: 在GitHub提交问题

---

**提示**: 如果你是ROS新手，建议先熟悉基本概念，然后再开始复杂项目。
