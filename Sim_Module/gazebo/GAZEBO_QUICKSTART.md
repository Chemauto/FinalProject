# 🚀 ROS2 + Gazebo 四足机器人仿真系统

基于 **ROS2 Humble** + **Gazebo** 的 3D 四足机器人仿真平台，支持 Unitree Go2 机器人，实现 Sim2Real 算法开发与验证。

> **注意**:
> - 本系统使用 Gazebo 11 / Gazebo Fortress（与 ROS2 Humble 完美兼容）
> - 包含自建的 `go2_gazebo_description` ROS2 包，无需下载 unitree_ros
> - **最新更新**: 修复了启动脚本，使用 `gazebo` 命令直接启动，避免 ros2 launch 兼容性问题

## 📋 目录

- [系统架构](#系统架构)
- [环境准备](#环境准备)
- [快速开始](#快速开始)
- [使用指南](#使用指南)
- [故障排除](#故障排除)

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    用户输入自然语言指令                       │
│                   "前进1米，然后左转90度"                     │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────┐
│            🧠 上层 LLM (qwen-plus) - 任务规划               │
│                 将复杂指令分解为子任务序列                    │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
       子任务 1: 前进1米    子任务 2: 左转90度
                       ↓
┌─────────────────────────────────────────────────────────────┐
│            ⚙️  下层 LLM (qwen-plus) - 任务执行               │
│                   调用 MCP 工具函数                           │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
              move_forward(1.0m)  turn_left(90°)
                       ↓
┌─────────────────────────────────────────────────────────────┐
│                    📤 ROS2 话题通信                          │
│              发布到 /robot_command (JSON)                    │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────┐
│         🤖 Gazebo Robot Controller (订阅 /robot_command)     │
│                   转换为 /cmd_vel 速度命令                    │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────────────┐
│              🎮 Gazebo 物理仿真 + Unitree 机器人              │
│                   实时动力学仿真与可视化                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔧 环境准备

### 系统要求

- **操作系统**: Ubuntu 22.04
- **Python**: 3.10 (ros2_env 环境)
- **ROS2**: Humble
- **内存**: 至少 8GB RAM
- **显卡**: 推荐 NVIDIA GPU（用于物理仿真加速）

### 安装步骤

#### 1. 安装 Gazebo

```bash
cd /home/robot/work/FinalProject/ROS_Module/ros2
./install_gazebo.sh
```

**安装内容**:
- Gazebo 11 仿真器（或 Gazebo Fortress，如果可用）
- ROS2-Gazebo 集成插件 (`ros-humble-gazebo-ros-pkgs`)
- Gazebo ROS2 Control 控制器

**验证安装**:
```bash
# Gazebo 11
gazebo --version
# 或 Gazebo Fortress
gz sim --version
```

#### 2. 编译 Go2 机器人模型

```bash
./build_unitree_model.sh
```

**编译内容**:
- 项目自带的 `go2_gazebo_description` ROS2 包
- Unitree Go2 机器人 URDF 模型
- 机器人控制器配置（diff_drive_controller）
- Gazebo 启动文件

**说明**:
- 使用项目自带的 Go2 描述包，无需下载 unitree_ros
- 自动跳过 ROS1 包的编译
- 简化的轮式模型，便于快速验证 Sim2Real 算法

#### 3. 激活 conda 环境

```bash
conda activate ros2_env
source /opt/ros/humble/setup.bash

# 加载工作空间（如果编译了 go2_gazebo_description）
source ~/ros2_ws/install/setup.bash
```

---

## 🚀 快速开始

### 启动 Gazebo 仿真系统

```bash
cd /home/robot/work/FinalProject/ROS_Module/ros2

# 激活环境
conda activate ros2_env
source /opt/ros/humble/setup.bash

# 启动 Gazebo 仿真
./start_ros2_mcp.sh --sim gazebo
```

**启动过程**:
1. 检查环境（conda + ROS2）
2. 启动 Gazebo 仿真环境（约 10-15 秒）
3. 加载 Go2 机器人模型（自动 spawn 到场景）
4. 启动机器人控制器（GazeboRobotController）
5. 启动交互式 MCP 界面

**预期结果**:
- ✅ Gazebo 窗口弹出，显示 3D 仿真环境
- ✅ Go2 机器人出现在场景中（蓝色车身，黑色轮子）
- ✅ 终端显示 "ROS2 MCP 系统已就绪"

**机器人模型说明**:
- 简化的轮式模型（便于快速验证 Sim2Real 算法）
- 蓝色车身，4 个黑色轮子
- 使用 diff_drive_controller 控制
- 与真实 Go2 相同的 ROS2 接口（/cmd_vel 话题）

### 测试指令

启动后，在交互界面输入以下指令：

```bash
# 基础运动
前进1米
后退50cm
左转90度
右转45度

# 组合指令（双层 LLM 的优势）
先左转90度，再往前走1米
前进50厘米然后向右转45度
左转90度，前进1米，然后右转45度

# 复杂任务
沿正方形路径走一圈
```

---

## 📖 使用指南

### 仿真模式对比

| 模式 | 命令 | 适用场景 | 启动时间 | 保真度 |
|------|------|---------|---------|--------|
| **2D仿真** | `--sim 2d` | 快速逻辑验证 | <1秒 | 低 |
| **Gazebo 3D** | `--sim gazebo` | 物理仿真、Sim2Real | ~15秒 | 高 |

| **真实机器人** | `--sim real` | 实际部署 | - | 100% |

### MCP 工具列表

系统支持以下 MCP 工具：

| 工具名 | 功能 | 参数 |
|--------|------|------|
| `move_forward` | 前进 | `distance`: 数值, `unit`: m/cm/mm |
| `move_backward` | 后退 | `distance`: 数值, `unit`: m/cm/mm |
| `turn_left` | 左转 | `angle`: 角度（度） |
| `turn_right` | 右转 | `angle`: 角度（度） |
| `pick_up` | 抓取 | `object_name`: 物体名称 |
| `place` | 放置 | `object_name`, `location` |
| `stop` | 停止 | - |

### ROS2 话题

**发布话题**:
- `/robot_command` - 接收 JSON 格式的机器人命令

**订阅话题**:
- `/cmd_vel` - 发送速度控制命令 (geometry_msgs/Twist)

**监控话题**:
```bash
# 查看命令
ros2 topic echo /robot_command

# 查看速度
ros2 topic echo /cmd_vel

# 可视化节点图
rqt_graph
```

---

## 🧪 测试流程

### 完整测试流程

#### 1. 环境检查

```bash
# 检查 conda 环境
conda activate ros2_env
python3 --version  # 应该显示 Python 3.10.19

# 检查 ROS2
source /opt/ros/humble/setup.bash
ros2 topic list  # 应该显示 ROS2 话题列表

# 检查 Gazebo
gazebo --version || gz sim --version  # 应该显示 Gazebo 版本
```

#### 2. 启动仿真

```bash
cd /home/robot/work/FinalProject/ROS_Module/ros2
./start_ros2_mcp.sh --sim gazebo
```

#### 3. 验证组件

**终端输出**:
```
✓ Gazebo (3D物理仿真环境)
✓ Unitree 四足机器人
✓ Gazebo Robot Controller (机器人控制)
✓ 交互式 MCP (命令行界面)
```

**Gazebo 窗口**:
- 显示 3D 场景
- Unitree 四足机器人加载

#### 4. 测试控制

```bash
# 测试 1: 前进
> 前进1米

# 预期: 机器人向前移动 1 米

# 测试 2: 转向
> 左转90度

# 预期: 机器人原地左转 90 度

# 测试 3: 组合指令
> 先左转90度，再往前走1米

# 预期:
# [上层 LLM] 分解为 2 个子任务
#   1. 向左转90度
#   2. 向前走1米
# [执行] 依次完成两个动作
```

#### 5. 检查日志

```bash
# Gazebo 日志
cat /tmp/gazebo_sim.log

# 控制器日志
cat /tmp/gazebo_controller.log
```

---

## 🐛 故障排除

### 问题 1: Gazebo 窗口未弹出

**症状**: 终端显示 "Gazebo 启动失败"

**解决**:
```bash
# 检查 Gazebo 是否安装
gazebo --version || gz sim --version

# 重新安装
cd /home/robot/work/FinalProject/ROS_Module/ros2
./install_gazebo.sh
```

### 问题 2: Gazebo 窗口打开但看不到机器人

**症状**: Gazebo 启动成功，但场景中没有 Go2 机器人

**原因**: robot_state_publisher 或 spawn_entity 失败

**解决**:
```bash
# 查看 spawn 日志
cat /tmp/spawn.log

# 查看 robot_state_publisher 日志
cat /tmp/rsp.log

# 手动 spawn 机器人
ros2 run gazebo_ros spawn_entity.py \
    -entity go2 \
    -topic /robot_description \
    -x 0.0 -y 0.0 -z 0.3
```

### 问题 3: 机器人不响应命令

**症状**: 输入指令后机器人不动

**解决**:
```bash
# 检查话题
ros2 topic list | grep cmd_vel

# 检查控制器是否运行
ps aux | grep gazebo_robot_controller

# 查看日志
cat /tmp/gazebo_controller.log

# 检查 /cmd_vel 话题是否在发布
ros2 topic hz /cmd_vel
```

### 问题 4: ModuleNotFoundError: rclpy

**症状**: `No module named 'rclpy'`

**解决**:
```bash
# 激活 ros2_env 环境
conda activate ros2_env

# 加载 ROS2
source /opt/ros/humble/setup.bash
```

### 问题 5: Go2 模型加载失败

**症状**: "未找到 go2_gazebo_description 包"

**解决**:
```bash
# 检查包是否存在
ls ROS_Module/ros2/go2_gazebo_description/

# 重新编译
cd /home/robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_gazebo_description

# 加载工作空间
source ~/ros2_ws/install/setup.bash

# 验证包已安装
ros2 pkg list | grep go2_gazebo
```

### 问题 6: 清理残留进程

**症状**: 重新启动时端口冲突

**解决**:
```bash
# 清理 Gazebo 进程
pkill -f gz-sim
pkill -f "gz server"

# 清理控制器
pkill -f gazebo_robot_controller

# 清理 ROS2 节点
pkill -f ros2
```

---

## 📁 文件结构

```
ROS_Module/ros2/
├── GAZEBO_QUICKSTART.md         # 本文档
├── install_gazebo.sh             # Gazebo 安装脚本
├── build_unitree_model.sh       # Go2 模型编译脚本
├── start_ros2_mcp.sh            # 总启动脚本（支持多模式）
├── start_gazebo_simple.sh       # 简化的 Gazebo 启动脚本 ✨
├── go2_sim_launch.py            # Gazebo 启动文件（备用）
├── gazebo_robot_controller.py   # Gazebo 机器人控制器
├── ros2_interactive_mcp.py      # 双层 LLM 交互系统
├── ros2_robot_controller.py     # 2D/真实机器人控制器
├── ros2_simulator.py            # 2D 仿真器
└── go2_gazebo_description/      # Go2 机器人 ROS2 包
    ├── package.xml              # ROS2 包描述
    ├── CMakeLists.txt           # CMake 构建配置
    ├── setup.py                 # Python 包配置
    ├── launch/                  # 启动文件
    │   └── go2_gazebo.launch.py
    ├── urdf/                    # 机器人 URDF
    │   └── go2.urdf.xacro
    ├── config/                  # 控制器配置
    │   └── controllers.yaml
    └── README.md                # 包文档
```

---

## 🎯 Sim2Real 开发路径

```
1️⃣  2D仿真 (pygame)
        ↓ 快速验证算法逻辑
2️⃣  Gazebo 3D仿真 ← 当前阶段
        ↓ 物理验证 + 参数调优
3️⃣  真实机器人部署
        ↓ 现场测试与迭代
4️⃣ 优化后的算法
        ↓ 回到 2️⃣ 或 3️⃣
```

---

## 🔗 相关资源

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [Gazebo 官方文档](https://gazebosim.org/docs)
- [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros)
- [项目根目录](../../README.md)

---

## ✅ 快速检查清单

启动前确认：

- [ ] `ros2_env` 环境已激活
- [ ] ROS2 已加载 (`source /opt/ros/humble/setup.bash`)
- [ ] Gazebo 已安装 (`gazebo --version` 或 `gz sim --version`)
- [ ] Go2 模型已编译 (`ls ~/ros2_ws/install`)
- [ ] `.env` 文件包含 `Test_API_KEY`

**一键检查**:
```bash
conda activate ros2_env && \
source /opt/ros/humble/setup.bash && \
(gazebo --version || gz sim --version) && \
source ~/ros2_ws/install/setup.bash && \
ros2 pkg list | grep go2_gazebo && \
python3 -c "import rclpy; print('✓ 环境就绪')"
```

---

## 📝 更新日志

### 2026-01-10 - 重大修复

**问题**: Gazebo 启动失败，机器人模型无法 spawn 到场景中

**原因**:
- `ros2 launch ROS_Module` 命令失败（ROS_Module 不是 ROS2 包）
- gazebo_ros 插件与 Gazebo 11 存在兼容性问题

**解决方案**:
1. ✅ 修改启动脚本，使用 `gazebo` 命令直接启动（不使用 ros2 launch）
2. ✅ 添加 robot_state_publisher 自动启动
3. ✅ 使用 spawn_entity.py 自动 spawn 机器人到场景
4. ✅ 增强清理函数，退出时清理所有相关进程
5. ✅ 添加 `start_gazebo_simple.sh` 作为备用启动脚本

**改进后的启动流程**:
```bash
# 1. 直接启动 Gazebo（空世界）
gazebo /opt/ros/humble/share/gazebo_ros/worlds/empty.world

# 2. 启动 robot_state_publisher（发布机器人状态）
ros2 run robot_state_publisher robot_state_publisher

# 3. Spawn 机器人到 Gazebo 场景中
ros2 run gazebo_ros spawn_entity.py -entity go2 -topic /robot_description

# 4. 启动 MCP 交互界面
python3 ros2_interactive_mcp.py
```

**新增文件**:
- `start_gazebo_simple.sh` - 简化的 Gazebo 启动脚本（便于调试）

**修复的问题**:
- ❌ `Package 'ROS_Module' not found` → ✅ 使用 gazebo 命令直接启动
- ❌ `gzserver process has died` → ✅ 使用更稳定的启动方式
- ❌ 机器人模型不显示 → ✅ 自动 spawn 到场景

---

## 🎉 开始使用

环境就绪后，运行：

```bash
conda activate ros2_env
cd /home/robot/work/FinalProject/ROS_Module/ros2
./start_ros2_mcp.sh --sim gazebo
```

享受 3D 四足机器人仿真的乐趣！🚀
