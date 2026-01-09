# ROS_Module - ROS2 机器人控制模块

用于控制真实机器人的 ROS2 模块，支持双层 LLM 架构。

## 🎯 模块概述

本模块提供：
- **ROS2 Robot Controller** - 接收并执行机器人命令
- **ROS2 Simulator** - 可视化仿真界面（类似 Dora）
- **双层 LLM 交互系统** - 任务规划 + 执行
- **一键启动脚本** - 自动化启动所有组件

**✨ 新特性：现在支持可视化仿真界面！**

## 📁 文件结构

```
ROS_Module/
└── ros2/
    ├── ros2_interactive_mcp.py      # 双层 LLM 交互系统
    ├── ros2_robot_controller.py     # ROS2 机器人控制器
    ├── ros2_simulator.py            # 可视化仿真器
    └── start_ros2_mcp.sh            # 一键启动脚本
```

## 🚀 快速开始

### 1. 安装 ROS2 Humble

```bash
# 添加 ROS2 软件源
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2
sudo apt update
sudo apt install ros-humble-desktop -y
```

### 2. 配置环境

```bash
# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 添加到 .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. 创建 Python 环境（重要）

**ROS2 Humble 需要 Python 3.10**

```bash
# 创建 Python 3.10 conda 环境
conda create -n ros2_env python=3.10 -y
conda activate ros2_env

# 安装依赖
pip install mcp openai python-dotenv pygame
```

### 4. 一键启动

```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash
cd ROS_Module/ros2
./start_ros2_mcp.sh
```

**就这么简单！** 系统会自动：
- ✅ 检查环境和依赖
- ✅ 启动 ROS2 Simulator（可视化仿真窗口）
- ✅ 启动 ROS2 Robot Controller（后台）
- ✅ 启动交互式 MCP（前台）
- ✅ 退出时自动清理

**你会看到：**
1. 一个可视化仿真窗口（显示机器人位置和运动）
2. 终端命令行界面（输入指令）

## 💬 测试指令

启动后直接输入：

### 基础指令
```
前进1米
左转90度
向右转45度
后退50cm
```

### 多步指令（双层 LLM 的优势）
```
先左转90度，再往前走1米
前进50厘米然后向右转45度
左转90度，前进1米，然后右转45度
```

### 操作指令
```
抓取杯子
把杯子放在桌子上
```

## 🔍 执行流程示例

输入：`先左转90度，再往前走1米`

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🧠 [上层LLM] 任务规划中...
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ [规划完成] 共分解为 2 个子任务
📋 [任务概述] 左转后前进

子任务序列：
  步骤 1: 向左转90度 (转向)
  步骤 2: 向前走1米 (移动)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🚀 [开始执行] 按顺序执行子任务
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【步骤 1/2】
🔧 [工具调用] turn_left({'angle': 90})
📤 [ROS2] 发送命令: {'action': 'turn_left', 'parameters': {'angle': '90deg'}}
⏳ [等待] 执行时间: 2.0秒.. ✅ 完成!

【步骤 2/2】
🔧 [工具调用] move_forward({'distance': 1.0, 'unit': 'm'})
📤 [ROS2] 发送命令: {'action': 'navigate', 'parameters': {'direction': 'front', 'distance': '1.0m'}}
⏳ [等待] 执行时间: 2.0秒.. ✅ 完成!

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ [执行完成] 任务总结
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. 向左转90度 - ✅ 成功
  2. 向前走1米 - ✅ 成功
```

## 🔧 手动分步启动（调试用）

如果需要分别查看各个组件的日志：

**终端 1 - ROS2 Robot Controller:**
```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash
python3 ros2_robot_controller.py
```

**终端 2 - 交互式 MCP:**
```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash
python3 ros2_interactive_mcp.py
```

## 📊 ROS2 话题

- **订阅**: `/robot_command` - 接收机器人命令
- **发布**: `/cmd_vel` - 控制机器人运动

### 监控话题

```bash
# 查看命令话题
ros2 topic echo /robot_command

# 查看速度命令
ros2 topic echo /cmd_vel

# 可视化节点图
rqt_graph
```

## 🐛 常见问题

### 问题 1: ModuleNotFoundError: rclpy._rclpy_pybind11

**原因**: Python 版本不是 3.10

**解决**:
```bash
conda activate ros2_env
python3 --version  # 必须是 3.10
```

### 问题 2: ROS_DISTRO not set

**解决**:
```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO  # 应该显示 humble
```

### 问题 3: 节点无法通信

**解决**:
```bash
export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### 问题 4: API Key 未配置

**解决**:
在项目根目录创建 `.env` 文件：
```bash
Test_API_KEY=sk-your-api-key-here
```

获取 API Key: https://dashscope.aliyun.com

## 🎯 核心架构

```
┌─────────────────────────────────────────┐
│   用户输入: "先左转90度，再往前走1米"      │
└──────────────┬──────────────────────────┘
               ↓
┌──────────────────────────────────────────┐
│  🧠 上层LLM (qwen-plus) - 任务规划        │
│  将复杂指令分解为子任务序列                │
└──────────────┬──────────────────────────┘
               ↓
      子任务 1: 向左转90度
      子任务 2: 向前走1米
               ↓
┌──────────────────────────────────────────┐
│  ⚙️  下层LLM (qwen-plus) - 执行子任务     │
│  调用相应的MCP工具                         │
└──────────────┬──────────────────────────┘
               ↓
    turn_left(90) → move_forward(1.0m)
               ↓
┌──────────────────────────────────────────┐
│  📤 ROS2 发布到 /robot_command 话题       │
└──────────────┬──────────────────────────┘
               ↓
┌──────────────────────────────────────────┐
│  🤖 ROS2 Robot Controller 执行实际动作    │
└──────────────────────────────────────────┘
```

## 📚 相关资源

- [ROS2 官方教程](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 概念](https://docs.ros.org/en/humble/Concepts/Basic.html)
- [项目根目录](../README.md)
- [MCP_Server](../MCP_Server/README.md)

---

**提示**: 首次使用建议先在仿真环境测试，确认无误后再连接真实机器人。
