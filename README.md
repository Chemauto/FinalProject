# 🤖 FinalProject - 基于双层LLM的机器人控制系统

基于双层 LLM 架构的机器人控制系统，采用 MCP 模块化设计，使用 ROS2 话题通信。

## 🎯 项目概述

通过自然语言控制机器人执行复杂任务，支持追击、视觉检测等功能。

**核心特性：**
- 双层 LLM：任务规划 + 执行控制
- ROS2 通讯：标准化消息传递
- 自然语言交互：中文指令控制
- 2D 仿真：Pygame 可视化

## 🔄 系统流程

```
用户输入："追击敌人"
    ↓
[Interactive_Module] CLI交互界面
    ↓
[LLM_Module] 双层LLM
    ├─ 上层：任务规划 → 步骤序列
    └─ 下层：执行控制 → 工具调用
    ↓
[Robot_Module] MCP工具中心
    ├─ base.py: move_forward/turn/stop
    ├─ chase.py: chase_enemy
    └─ vision.py: detect_color
    ↓ ROS2
[Sim_Module] 2D仿真器 → 执行+可视化
```

## 📦 模块说明

### Interactive_Module - 交互界面

**作用**: CLI 交互，协调各模块

**输入**: `前进1米` / `追击敌人`
**输出**: 执行结果反馈

---

### LLM_Module - 双层LLM核心

**作用**: 任务规划和执行控制

**输入**: 用户指令 + 工具列表
**输出**: 子任务序列 / 工具调用

```python
agent = LLMAgent(api_key="...")
result = await agent.run_pipeline(user_input, tools, execute_fn)
```

---

### Robot_Module - MCP工具注册中心

**作用**: 注册和执行机器人工具

**输入**: `chase_enemy()` / `move_forward(1.0, 0.3)`
**输出**: ROS2动作命令

```json
{"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.3}}
```

**可用工具**:
- 基础: `move_forward`, `move_backward`, `turn`, `stop`
- 追击: `chase_enemy`, `chase_target`, `get_enemy_positions`
- 视觉: `detect_color_and_act`

---

### Sim_Module - 2D仿真环境

**作用**: 可视化仿真，执行动作

**输入**: ROS2 `/robot/command`
**输出**: 机器人运动 + 状态发布 `/robot/state`

**操作**: 鼠标生成敌人 / C清除 / ESC退出

---

### Test_Module - 追击功能

**作用**: 追击算法和测试仿真器

**核心**: `ChaseController`, `EnemyManager`, `ChaseSimulator`

**流程**: 获取敌人 → 计算最近 → PID控制 → 旋转前进 → 清除

**参数**: `MAX_STEP_DISTANCE=0.5m`, `ARRIVAL_THRESHOLD=5px`

---

### VLM_Module - 视觉语言模型

**作用**: 图像理解和颜色检测

**输入**: 图像路径
**输出**: 颜色 + 动作映射

```python
{"color": "红色", "action": "turn", "angle": -90.0}
```

**支持**: 本地Ollama / 远程API

---

### Yolo_Module - YOLO目标检测

**作用**: 屏幕捕获和目标检测

**输入**: 屏幕截图
**输出**: 检测结果

```python
[{"class": "person", "confidence": 0.95, "bbox": [x,y,w,h]}]
```

---

## 📡 ROS2通讯

| 话题 | 用途 |
|------|------|
| `/robot/command` | 动作命令 |
| `/robot/state` | 机器人状态 |
| `/robot/enemies` | 敌人位置 |
| `/robot/enemy_remove` | 清除敌人 |

## 🚀 快速开始

### 安装运行

```bash
pip install -r requirements.txt
export Test_API_KEY=your_key

# 终端1: 仿真器
python3 Sim_Module/sim2d/simulator.py

# 终端2: 交互程序
python3 Interactive_Module/interactive.py
```

### 使用示例

```bash
# 基础
前进1米 / 左转90度

# 追击
追击敌人 / 追击坐标(700,300)

# 视觉
根据 /path/to/image.png 检测颜色并执行动作

# 复杂
先左转45度，前进1.5米，然后右转90度
```

## 📝 依赖

```bash
openai>=1.0.0  fastmcp>=0.1.0  pygame>=2.5.0
pyyaml>=6.0  python-dotenv>=1.0.0  rclpy
```

---

**模块化，易扩展！** 🚀
