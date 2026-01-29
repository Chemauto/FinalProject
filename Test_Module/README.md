# IsaacSim 机器人控制模块

## 概述

本模块实现通过 Socket UDP 控制 IsaacSim 中的四足机器人（Unitree Go2）执行行走动作。采用与 `send_cmd.py` 一致的持续发送命令机制，将任务分解为多个小步骤执行。

## 架构设计

### 整体流程

```
用户输入 → Interactive_Module → LLM_Module (任务规划)
                                           ↓
                                    LLM_Module (工具执行)
                                           ↓
                              Robot_Module/walkisaacsim.py (MCP工具)
                                           ↓
                              Socket UDP (127.0.0.1:5555)
                                           ↓
                              IsaacSim (Unitree Go2 机器人)
```

### 核心组件

#### 1. IsaacSimSocketController（Socket控制器）

**功能**: 管理 UDP Socket 连接，发送速度命令

**特性**:
- 单例模式：确保只有一个 Socket 实例
- 持续连接：创建一次，重复使用
- 命令格式：`"vx,vy,wz"`（与 send_cmd.py 一致）

**核心方法**:
```python
send_command(vx, vy, wz)     # 发送速度命令
send_command_list([vx,vy,wz]) # 使用列表发送
stop()                        # 发送停止命令 [0,0,0]
```

#### 2. VelocityCommand（数据结构）

**作用**: 封装速度命令数据

```python
@dataclass
class VelocityCommand:
    vx: float  # 前进速度 (m/s)
    vy: float  # 横向速度 (m/s)
    wz: float  # 旋转角速度 (rad/s)
```

#### 3. move_isaac（MCP工具函数）

**功能**: 统一的移动控制接口

**参数**:
- `direction`: 移动方向（forward/backward/left/right/rotate_left/rotate_right）
- `distance`: 距离（米）或角度（度）
- `speed`: 速度（可选，默认使用各方向预设值）

**执行机制**:
```
1. 参数验证
2. 任务分解：计算步骤数 = 总时间 / 0.1秒
3. 循环执行：
   - 发送速度命令
   - 等待 0.1 秒
   - 打印进度（每10步）
4. 发送停止命令
```

### 任务分解机制

**原理**: 将长距离移动分解为多个小步骤，每步持续发送命令

**示例**（前进1米，速度0.5m/s）:
```
总时间 = 1.0m / 0.5m/s = 2秒
步骤数 = 2秒 / 0.1秒 = 20步
每步时间 = 0.1秒

执行过程:
步骤 1: 发送 [0.5, 0.0, 0.0], 等待 0.1s
步骤 2: 发送 [0.5, 0.0, 0.0], 等待 0.1s
...
步骤 20: 发送 [0.5, 0.0, 0.0], 等待 0.1s
停止: 发送 [0.0, 0.0, 0.0]
```

### 速度配置

**默认值**（与 `/home/xcj/work/IsaacLab/IsaacLabBisShe/Socket/send_cmd.py` 完全一致）:

| 方向 | 速度向量 [vx, vy, wz] | 说明 |
|------|---------------------|------|
| forward | [0.5, 0.0, 0.0] | 前进 0.5 m/s |
| backward | [-0.3, 0.0, 0.0] | 后退 0.3 m/s |
| left | [0.0, 0.5, 0.0] | 左移 0.5 m/s |
| right | [0.0, -0.5, 0.0] | 右移 0.5 m/s |
| rotate_left | [0.0, 0.0, 0.5] | 左转 0.5 rad/s |
| rotate_right | [0.0, 0.0, -0.5] | 右转 0.5 rad/s |

**坐标系说明**:
- `vx`: 前进速度（正值向前，负值向后）
- `vy`: 横向速度（正值向左，负值向右）
- `wz`: 旋转角速度（正值左转/逆时针，负值右转/顺时针）

## 使用说明

### 1. 启动 IsaacSim 环境

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe

# 启动训练好的策略
python scripts/rsl_rl/play.py \
    --task Template-Velocity-Go2-Walk-Flat-Ros-v0 \
    --checkpoint /home/xcj/work/IsaacLab/IsaacLabBisShe/ModelBackup/WalkPolicy/WalkFlatNew.pt
```

**预期行为**:
- IsaacSim 启动并加载 Go2 机器人模型
- SocketVelocityCommand 开始监听 UDP 端口 5555
- 机器人初始静止，等待 socket 命令

### 2. 启动交互系统

```bash
cd /home/xcj/work/FinalProject
python Interactive_Module/interactive.py
```

### 3. 发送控制指令

**示例指令**:
```
前进1米
后退0.5米
左移1米
右转90度
```

**执行流程**:
```
💬 请输入指令: 前进1米

[LLM规划] 分解为移动任务
[工具调用] move_isaac(direction="forward", distance=1.0)

[move_isaac] 任务分解: 20 步, 每步 0.100s, 总时间 2.00s
[move_isaac] 开始持续发送命令...
[move_isaac] 进度: 10% (2/20)
[move_isaac] 进度: 20% (4/20)
...
[move_isaac] 进度: 100% (20/20)
[move_isaac] 移动完成，已发送停止命令
```

## 技术细节

### Socket 通信协议

**协议**: UDP
**地址**: `127.0.0.1:5555`
**数据格式**: `"vx,vy,wz"`（字符串，保留3位小数）

**示例**:
```
"0.500,0.000,0.000"  # 前进
"-0.300,0.000,0.000" # 后退
"0.000,0.500,0.000"  # 左移
"0.000,0.000,0.500"  # 左转
"0.000,0.000,0.000"  # 停止
```

### 时间计算

**线移动**:
```
时间(秒) = 距离(米) / 速度(m/s)
```

**旋转**:
```
时间(秒) = 角度(度) × π/180 / 角速度(rad/s)
```

### 参数限制

| 参数 | 范围 | 说明 |
|------|------|------|
| distance (线移动) | 0 ~ 10 米 | 最大移动距离 |
| distance (旋转) | 0 ~ 360 度 | 最大旋转角度 |
| speed (线速度) | 0.1 ~ 2.0 m/s | 线速度范围 |
| speed (角速度) | 0.1 ~ 3.14 rad/s | 角速度范围 |
| step_interval | 0.1 秒 | 命令发送间隔 |

## 文件结构

```
/home/xcj/work/FinalProject/
├── Robot_Module/
│   ├── skill.py                    # MCP服务器入口
│   └── module/
│       └── walkisaacsim.py         # IsaacSim控制模块（本模块）
│
├── Interactive_Module/
│   └── interactive.py              # 交互界面
│
├── LLM_Module/
│   └── llm_core.py                 # LLM核心逻辑
│
└── Test_Module/
    └── model/
        └── WalkFlatNew.pt          # 训练好的策略模型

/home/xcj/work/IsaacLab/IsaacLabBisShe/
├── Socket/
│   └── send_cmd.py                 # 参考实现（键盘控制）
├── ModelBackup/WalkPolicy/
│   └── WalkFlatNew.pt              # 策略模型副本
└── scripts/rsl_rl/
    └── play.py                     # IsaacSim启动脚本
```

## 与 send_cmd.py 的关系

### 相同点
1. **通信协议**: 都使用 UDP Socket 端口 5555
2. **命令格式**: 都使用 `"vx,vy,wz"` 格式
3. **速度值**: 使用完全相同的默认速度值
4. **持续发送**: 都采用持续发送命令机制

### 不同点
| 特性 | send_cmd.py | walkisaacsim.py |
|------|-------------|-----------------|
| 控制方式 | 键盘实时控制 | 程序化自动控制 |
| 任务分解 | 无（手动控制） | 有（自动分解） |
| 停止方式 | 按键释放/Space | 自动计算后停止 |
| 使用场景 | 手动测试 | LLM自动控制 |

## 故障排查

### Q1: 机器人不移动

**检查步骤**:
1. 确认 IsaacSim 已启动并显示机器人
2. 检查端口 5555 是否被占用: `netstat -tuln | grep 5555`
3. 查看日志输出是否有 "Socket 已初始化"
4. 确认模型路径正确

### Q2: 移动距离不准确

**可能原因**:
- 速度与实际机器人运动速度不匹配
- 地面摩擦力影响

**解决方法**:
- 调整 speed 参数进行校准
- 使用较小的距离值进行测试

### Q3: 命令发送失败

**检查**:
```python
# 测试 Socket 连接
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b"0.5,0.0,0.0", ("127.0.0.1", 5555))
```

## 参考文档

- [Isaac Lab 官方文档](https://isaac-sim.github.io/IsaacLab/main/)
- Socket 实现说明: `/home/xcj/work/IsaacLab/IsaacLabBisShe/Socket/README.md`
- 策略训练说明: `/home/xcj/work/IsaacLab/IsaacLabBisShe/TestREADME.md`

## 更新日志

### v2.0 (2025-01-29)
- 重构为持续发送命令机制
- 实现任务分解功能
- 与 send_cmd.py 保持一致的速度值
- 添加进度显示功能

### v1.0 (初始版本)
- 基于 ROS2 话题通信
- 基本的移动控制功能
