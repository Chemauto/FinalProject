# Go2 MuJoCo 仿真控制

## 文件结构

```
Go2_Quadruped/
├── go2_description/          # Go2 机器人描述文件
│   ├── xacro/               # Xacro 文件
│   ├── urdf/                # URDF 文件
│   ├── meshes/              # 网格文件
│   └── launch/              # Launch 文件
├── skills/                   # 机器人技能
│   ├── __init__.py
│   └── go2_skills.py        # Go2 技能实现
├── robot_config.yaml        # 机器人配置
└── __init__.py              # 模块导出
```

## 快速开始

### 使用 ROS2 MCP 系统 (推荐)

```bash
cd /home/xcj/work/FinalProject/Middle_Module/ROS
./start_ros2_mcp.sh --sim mujoco
```

### 使用 MuJoCo 仿真器直接启动

```bash
cd /home/xcj/work/FinalProject/Sim_Module/mujoco
./start_mujoco_sim.sh
```

## 控制方式

### 1. 使用 cmd_vel 话题

```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# 后退
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.2}, angular: {z: 0.0}}"

# 左转
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# 右转
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"

# 停止
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 2. 使用 ROS2 MCP 自然语言控制

启动 ROS2 MCP 交互式系统后:

```
👤 用户> 前进1米
👤 用户> 左转90度
👤 用户> 向右移动50厘米
👤 用户> 挥手打招呼
```

## 可用技能

| 技能名称 | 描述 | 参数 |
|---------|------|------|
| `move_forward` | 向前移动 | distance (米), speed (m/s) |
| `move_backward` | 向后移动 | distance (米), speed (m/s) |
| `turn_left` | 左转 | angle (度), angular_speed |
| `turn_right` | 右转 | angle (度), angular_speed |
| `strafe_left` | 左侧移动 (螃蟹步) | distance (米), speed |
| `strafe_right` | 右侧移动 (螃蟹步) | distance (米), speed |
| `stop` | 停止 | - |
| `stand` | 站立姿态 | - |
| `sit` | 蹲下姿态 | - |
| `wave` | 挥手打招呼 | duration (秒) |

## MuJoCo 仿真器说明

MuJoCo 仿真器位于 `Sim_Module/mujoco/mujoco_simulator.py`，实现了:

1. **cmd_vel 订阅**: 接收几何消息 Twist 进行速度控制
2. **步态生成**: 生成 Trot 步态 (对角步态)
3. **物理仿真**: MuJoCo 高精度物理引擎
4. **实时可视化**: MuJoCo Viewer 3D 显示

### 控制模式

- **stand**: 站立姿态，保持稳定
- **trot**: Trot 步态行走，根据 cmd_vel 调整

## 关节名称

Go2 有 12 个关节 (每条腿 3 个):

```
FL_hip, FL_thigh, FL_calf  (前左)
FR_hip, FR_thigh, FR_calf  (前右)
RL_hip, RL_thigh, RL_calf  (后左)
RR_hip, RR_thigh, RR_calf  (后右)
```

## 调试

### 查看关节状态

```bash
ros2 topic echo /joint_states
```

### 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

### 查看 MuJoCo 日志

```bash
cat /tmp/mujoco_simulator.log
```

## MuJoCo vs Gazebo

| 特性 | MuJoCo | Gazebo |
|------|--------|--------|
| 仿真速度 | ⚡ 5-10倍更快 | 较慢 |
| 物理精度 | 🎯 接触模型更准确 | 一般 |
| 安装难度 | 🔧 纯 pip 安装 | 复杂依赖 |
| ROS2 集成 | ✅ 简单 | ❌ 插件问题多 |
| 内存占用 | 💪 小 | 大 |
| 适合场景 | 强化学习、Sim2Real | 复杂场景仿真 |

## 注意事项

1. **启动顺序**: 系统会自动启动所有组件
2. **速度限制**: 线速度建议 0.1-0.5 m/s，角速度建议 0.3-1.0 rad/s
3. **清理**: 使用 Ctrl+C 退出会自动清理所有进程

## 依赖

- ROS2 Humble/Jazzy
- MuJoCo 3.4+
- Python 3.10+

## 参考资源

- [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros)
- [Go2 Description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Python API](https://github.com/google-deepmind/mujoco)
