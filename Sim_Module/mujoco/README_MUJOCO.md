# MuJoCo 仿真模块

## 概述

MuJoCo (Multi-Joint dynamics with Contact) 是一个高性能的物理仿真引擎，特别适合：

- 四足机器人动力学仿真
- 强化学习训练
- Sim2Real 研究
- 快速原型开发

与 Gazebo 相比，MuJoCo 的优势：
- ⚡ **更快速**：仿真速度比 Gazebo 快 5-10 倍
- 🎯 **更准确**：接触模型更精确，收敛性更好
- 🔧 **更简单**：纯 Python 实现，无需复杂的 ROS2 插件
- 💪 **更稳定**：内存占用小，运行更稳定

## 目录结构

```
Sim_Module/mujoco/
├── mujoco_simulator.py      # MuJoCo 仿真器主程序
├── install_mujoco.sh         # MuJoCo 安装脚本
├── start_mujoco_sim.sh       # MuJoCo 启动脚本
└── README_MUJOCO.md          # 本文档
```

## 快速开始

### 1. 安装 MuJoCo

```bash
cd /home/xcj/work/FinalProject/Sim_Module/mujoco
chmod +x install_mujoco.sh
./install_mujoco.sh
```

这会自动安装：
- `mujoco` - MuJoCo 物理引擎
- `glfw` - 窗口管理
- `PyOpenGL` - OpenGL 渲染
- 其他必需依赖

### 2. 启动 MuJoCo 仿真

有两种启动方式：

#### 方式 A：直接启动（推荐调试）

```bash
cd /home/xcj/work/FinalProject/Sim_Module/mujoco
chmod +x start_mujoco_sim.sh
./start_mujoco_sim.sh
```

#### 方式 B：通过 ROS2 MCP 系统启动

```bash
cd /home/xcj/work/FinalProject/Middle_Module/ROS
./start_ros2_mcp.sh --sim mujoco
```

### 3. 控制机器人

MuJoCo 仿真器通过 `/cmd_vel` 话题接收速度命令：

```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' --once

# 转向
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}' --once

# 停止
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once
```

## 机器人模型

仿真器包含一个简化的 Go2 四足机器人模型：

- **12 个关节**：每条腿 3 个（hip、thigh、calf）
- **Trot 步态**：对角步态，适合快速移动
- **站立姿态**：稳定的四足站立
- **物理参数**：基于 Unitree Go2 的近似参数

### 关节命名

```
FL: 前左腿 (Front Left)
FR: 前右腿 (Front Right)
RL: 后左腿 (Rear Left)
RR: 后右腿 (Rear Right)

每条腿的关节：
- hip: 髋关节（旋转）
- thigh: 大腿关节（俯仰）
- calf: 小腿关节（俯仰）
```

### 步态模式

默认使用 **Trot 步态**（对角步态）：
- FL 和 RR 同相
- FR 和 RL 同相
- 两条对角腿相位差 180°

步态参数：
- 频率：2.0 Hz
- 步高：0.05 m
- 步长：0.1 m

## 技术细节

### 仿真器架构

```python
Go2MuJoCoSimulator
├── MuJoCo 模型加载
│   └── 内置 MJCF XML 模型
├── ROS2 集成
│   └── Go2MuJoCoNode (订阅 /cmd_vel)
├── 步态生成
│   └── generate_trot_gait()
└── 可视化
    └── MuJoCo Viewer
```

### 控制流程

```
用户命令 → /cmd_vel 话题
    ↓
Go2MuJoCoNode 接收命令
    ↓
generate_trot_gait() 生成关节角度
    ↓
mujoco.mj_step() 步进仿真
    ↓
MuJoCo Viewer 显示 3D 场景
```

### 时间步长

- 仿真步长：0.02s (50Hz)
- 物理子步：10 步/仿真步
- 控制频率：50Hz

## 故障排查

### MuJoCo 未安装

```bash
ModuleNotFoundError: No module named 'mujoco'
```

**解决方案**：
```bash
cd /home/xcj/work/FinalProject/Sim_Module/mujoco
./install_mujoco.sh
```

### 无法打开显示

```bash
Failed to create GLFW window
```

**解决方案**：
1. 检查 X11 转发（如果使用 SSH）：
```bash
ssh -X user@hostname
```

2. 检查 DISPLAY 环境变量：
```bash
echo $DISPLAY
```

### ROS2 节点未启动

**解决方案**：
确保 ROS2 环境已加载：
```bash
source /opt/ros/jazzy/setup.bash  # 或 humble
```

## 性能优化

### 提高仿真速度

1. **减少渲染质量**（在 mujoco_simulator.py 中）：
```python
# 减少子步数
mujoco.mj_step(self.model, self.data, nstep=5)  # 默认 10
```

2. **关闭可视化**（纯物理仿真）：
```python
# 不启动 viewer，直接运行循环
while True:
    self.update(dt=0.02)
```

### 强化学习训练

MuJoCo 非常适合 RL 训练：

```python
import mujoco
import numpy as np

# 批量仿真
for episode in range(1000):
    # 重置环境
    mujoco.mj_reset(self.model, self.data)

    # 运行 episode
    for step in range(1000):
        action = policy.get_action(obs)
        self.apply_action(action)
        mujoco.mj_step(self.model, self.data)
```

## 扩展开发

### 修改机器人模型

编辑 `mujoco_simulator.py` 中的 `_create_simple_model()` �：

```python
def _create_simple_model(self):
    xml_model = """
    <mujoco>
        <world>
            <body name="trunk" pos="0 0 0.5">
                <!-- 在这里修改机器人结构 -->
            </body>
        </world>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml_model)
```

### 添加传感器

```python
# 在 MJCF 中添加传感器
<sensor>
    <touch name="foot_touch" site="FL_foot"/>
    <accelerometer name="imu" site="trunk_imu"/>
</sensor>
```

### 自定义步态

```python
def generate_custom_gait(self, dt, linear_x, angular_z):
    """实现自定义步态"""
    # 你的步态生成代码
    return joint_positions
```

## 参考资料

- [MuJoCo 官方文档](https://mujoco.readthedocs.io/)
- [MuJoCo GitHub](https://github.com/google-deepmind/mujoco)
- [Unitree Go2 官方网站](https://www.unitree.com/go2/)

## 许可证

本项目遵循项目的整体许可证。

## 贡献

欢迎提交 Issue 和 Pull Request！

## 作者

- Claude Code (MuJoCo 集成)

---

**最后更新**: 2026-01-11
