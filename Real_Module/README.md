# Real_Module - 真实机器人模块

## 概述

Real_Module 提供了**真实机器人硬件驱动接口**，用于控制实际的物理机器人。该模块与仿真模块 (Sim_Module) 接口兼容，可以无缝切换。

### 核心功能

- **硬件驱动**: 机器人底层硬件控制接口
- **传感器数据**: 摄像头、激光雷达、IMU 等传感器数据采集
- **执行器控制**: 电机、舵机、夹爪等执行器控制
- **安全机制**: 碰撞检测、急停保护、边界限制

### 设计目标

1. **与仿真兼容**: 接口与 Sim_Module 保持一致
2. **硬件抽象**: 屏蔽底层硬件差异
3. **安全第一**: 内置多层安全保护机制
4. **易于扩展**: 支持多种机器人平台

## 目录结构

```
Real_Module/
├── __init__.py           # 模块导出
├── go2/                  # Unitree Go2 四足机器人
│   ├── __init__.py
│   ├── go2_driver.py     # Go2 驱动程序
│   └── config/           # Go2 配置文件
├── industrial/           # 工业机器人
│   └── __init__.py
├── mobile/               # 移动机器人
│   └── __init__.py
└── safety/               # 安全机制
    ├── __init__.py
    ├── collision_detection.py
    └── emergency_stop.py
```

## 支持的机器人平台

### 已实现

| 机器人 | 状态 | 说明 |
|-------|------|------|
| - | 计划中 | 等待硬件接入 |

### 计划支持

- **Unitree Go2**: 四足机器人 (优先)
- **Unitree A1**: 四足机器人
- **Differential Drive**: 差速移动机器人
- **Industrial Arm**: 工业机械臂

## 硬件接口规范

### 标准接口

```python
class RobotDriver:
    """机器人驱动基类"""

    def connect(self) -> bool:
        """连接到机器人硬件"""
        pass

    def disconnect(self) -> bool:
        """断开连接"""
        pass

    def is_connected(self) -> bool:
        """检查连接状态"""
        pass

    def get_state(self) -> dict:
        """获取机器人状态"""
        pass

    def send_command(self, command: dict) -> bool:
        """发送控制命令"""
        pass

    def emergency_stop(self) -> bool:
        """紧急停止"""
        pass
```

### ROS2 集成

真实机器人通过 ROS2 与系统通信，与仿真器使用相同的话题：

| 话题 | 方向 | 消息类型 | 说明 |
|------|------|----------|------|
| `/cmd_vel` | 订阅 | `geometry_msgs/Twist` | 速度命令 |
| `/sensor/odom` | 发布 | `nav_msgs/Odometry` | 里程计数据 |
| `/sensor/camera` | 发布 | `sensor_msgs/Image` | 摄像头图像 |
| `/sensor/lidar` | 发布 | `sensor_msgs/LaserScan` | 激光雷达数据 |
| `/sensor/imu` | 发布 | `sensor_msgs/Imu` | IMU 数据 |

## 传感器支持

### 视觉传感器

- **RGB 摄像头**: 彩色图像采集
- **深度相机**: 深度信息采集
- **全景相机**: 360° 环视

### 距离传感器

- **激光雷达**: 2D/3D 激光扫描
- **超声波传感器**: 近距离障碍检测
- **ToF 传感器**: 飞行时间测距

### 姿态传感器

- **IMU**: 惯性测量单元
- **磁力计**: 方向感知
- **编码器**: 里程测量

## 安全机制

### 1. 碰撞检测

```python
class CollisionDetector:
    """碰撞检测系统"""

    def check_collision(self, sensor_data: dict) -> bool:
        """检测是否即将发生碰撞"""
        # 分析传感器数据
        # 预测碰撞风险
        pass

    def get_safe_distance(self) -> float:
        """获取安全距离"""
        pass
```

### 2. 急停保护

```python
class EmergencyStop:
    """紧急停止系统"""

    def trigger(self, reason: str):
        """触发急停"""
        # 立即停止所有运动
        # 记录急停原因
        # 通知监控系统
        pass

    def reset(self):
        """重置急停状态"""
        pass
```

### 3. 边界限制

- **地理围栏**: 限制运动范围
- **速度限制**: 限制最大速度
- **力矩限制**: 保护关节和执行器

## 使用示例

### 基础使用

```python
from Real_Module.go2 import Go2Driver

# 初始化驱动
driver = Go2Driver()

# 连接到机器人
if driver.connect():
    print("✅ 连接成功")

    # 获取状态
    state = driver.get_state()
    print(f"位置: {state['position']}")
    print(f"电池: {state['battery']}%")

    # 发送命令
    command = {
        'action': 'move_forward',
        'parameters': {'speed': 0.3, 'distance': 1.0}
    }
    driver.send_command(command)

    # 断开连接
    driver.disconnect()
```

### 通过 ROS2 控制

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RealRobotController(Node):
    def __init__(self):
        super().__init__('real_robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, speed=0.3):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)

# 使用
rclpy.init()
controller = RealRobotController()

# 向前移动
controller.move_forward(0.3)
rclpy.spin_once(controller, timeout_sec=2.0)

# 停止
controller.stop()
```

## 配置文件

### 机器人配置示例

```yaml
# config/go2_config.yaml
robot:
  name: "Go2"
  type: "quadruped"
  manufacturer: "Unitree"

hardware:
  connection:
    type: "wifi"
    ip: "192.168.123.1"
    port: 8080

  sensors:
    - type: "camera"
      topic: "/sensor/camera"
    - type: "lidar"
      topic: "/sensor/lidar"
    - type: "imu"
      topic: "/sensor/imu"

  safety:
    max_velocity: 1.0
    max_acceleration: 2.0
    emergency_stop: true
    collision_detection: true
```

## 开发指南

### 添加新的机器人平台

1. **创建驱动目录**
   ```bash
   mkdir -p Real_Module/myrobot
   touch Real_Module/myrobot/__init__.py
   ```

2. **实现驱动类**
   ```python
   # Real_Module/myrobot/myrobot_driver.py
   from ..base import RobotDriver

   class MyRobotDriver(RobotDriver):
       def connect(self):
           # 实现连接逻辑
           pass

       def disconnect(self):
           # 实现断开逻辑
           pass
   ```

3. **添加配置文件**
   ```yaml
   # Real_Module/myrobot/config.yaml
   robot:
     name: "MyRobot"
     type: "custom"
   ```

4. **在主系统中注册**
   ```python
   # 在 Robot_Module 中添加配置
   ```

## 故障排除

### 连接失败

1. 检查网络连接
2. 确认 IP 地址正确
3. 检查防火墙设置
4. 验证机器人是否开机

### 传感器数据异常

1. 检查传感器连接
2. 验证话题是否正确
3. 使用 `ros2 topic echo` 检查数据流
4. 查看传感器驱动日志

### 机器人无响应

1. 检查急停状态
2. 验证通信链路
3. 查看错误日志
4. 尝试重启机器人

## 依赖

### 通用依赖

```
rclpy                    # ROS2 Python 客户端
python-dotenv            # 环境变量管理
pyyaml                   # 配置文件解析
```

### 特定机器人依赖

根据具体机器人平台，可能需要：
- Unitree SDK (Go2 机器人)
- 厂商提供的驱动程序
- 专用通信库

## 当前状态

⚠️ **开发中**: 模块框架已搭建，等待实际硬件接入

### 待实现功能

- [ ] Go2 四足机器人驱动
- [ ] 传感器数据采集
- [ ] 安全保护机制
- [ ] 错误诊断和恢复
- [ ] 远程监控界面

### 计划功能

- [ ] 更多机器人平台支持
- [ ] 传感器标定工具
- [ ] 性能监控和日志
- [ ] 远程升级功能

## 相关文档

- [Sim_Module README](../Sim_Module/README.md) - 仿真模块
- [Middle_Module README](../Middle_Module/README.md) - 通信层
- [Robot_Module README](../Robot_Module/README.md) - 机器人模块
- [主项目 README](../README.md) - 项目总览
