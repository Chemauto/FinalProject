# Go2 Gazebo Description

Unitree Go2 机器人的 Gazebo 仿真描述包（ROS2 版本）

## 包结构

```
go2_gazebo_description/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS2 包描述
├── setup.py                # Python 包配置
├── config/                 # 控制器配置
│   └── controllers.yaml
├── launch/                 # 启动文件
│   └── go2_gazebo.launch.py
├── meshes/                 # 3D 模型文件（预留）
├── resource/               # 包资源标记
└── urdf/                   # 机器人 URDF 描述
    └── go2.urdf.xacro
```

## 安装方法

### 方法 1: 使用项目自带脚本

```bash
cd /home/robot/work/FinalProject/ROS_Module/ros2
./build_unitree_model.sh
```

### 方法 2: 手动安装

```bash
# 创建 ROS2 工作空间（如果不存在）
mkdir -p ~/ros2_ws/src

# 复制本包到工作空间
cp -r /path/to/FinalProject/ROS_Module/ros2/go2_gazebo_description ~/ros2_ws/src/

# 编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_gazebo_description

# 加载环境
source ~/ros2_ws/install/setup.bash
```

## 使用方法

### 启动 Gazebo 仿真

```bash
# 方式 1: 使用项目启动脚本
cd /home/robot/work/FinalProject/ROS_Module/ros2
./start_ros2_mcp.sh --sim gazebo

# 方式 2: 直接启动 Gazebo
ros2 launch go2_gazebo_description go2_gazebo.launch.py
```

### 控制机器人

```bash
# 发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# 查看机器人状态
ros2 topic echo /odom
ros2 topic echo /joint_states
```

## 机器人参数

- **质量**: 15 kg (base)
- **轮子半径**: 0.05 m
- **轮距**: 0.24 m
- **最大线速度**: 1.0 m/s
- **最大角速度**: 1.0 rad/s

## 控制器

本包使用 `diff_drive_controller` 实现差速驱动控制：

- **左轮关节**: front_left_wheel_joint, back_left_wheel_joint
- **右轮关节**: front_right_wheel_joint, back_right_wheel_joint
- **命令话题**: /cmd_vel (geometry_msgs/Twist)
- **里程计话题**: /odom (nav_msgs/Odometry)

## 依赖

- ROS2 Humble
- Gazebo 11 / Gazebo Fortress
- ros-humble-gazebo-ros-pkgs
- ros-humble-ros2-control
- ros-humble-diff-drive-controller

## Sim2Real

本包是为 Sim2Real 开发设计的简化模型：

1. **简化设计**: 使用轮式模型代替四足，便于快速验证
2. **相同接口**: 与真实 Go2 机器人使用相同的 ROS2 话题接口
3. **易于扩展**: 可以逐步添加更复杂的四足动力学模型

## 后续改进

- [ ] 添加真实的四足 URDF 模型
- [ ] 添加 IMU 和摄像头传感器
- [ ] 添加激光雷达传感器
- [ ] 实现四足步态控制器

## 许可证

MIT License
