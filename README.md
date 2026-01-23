# 🤖 FinalProject - 基于双层LLM的机器人控制系统

基于双层 LLM 架构的机器人控制系统，采用 MCP 模块化设计，使用 ROS2 话题通信。

## 🎯 项目概述

通过自然语言控制机器人执行复杂任务，支持追击、视觉检测等功能。

**核心特性：**
- 双层 LLM：任务规划 + 执行控制
- ROS2 通讯：标准化消息传递
- 自然语言交互：中文指令控制
- 2D 仿真：Pygame 可视化
- 自适应追击：智能角度校正

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
    ├─ chase.py: chase_enemy (2个MCP工具)
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

**特性**：
- 支持任务分解（复杂任务 → 子任务序列）
- 支持 previous_result 传递（步骤间数据传递）
- 强制执行规则（追击任务必须使用上一步结果）

---

### Robot_Module - MCP工具注册中心

**作用**: 注册和执行机器人工具

**输入**: `chase_enemy()` / `move_forward(1.0, 0.3)`
**输出**: ROS2动作命令

```json
{"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.3}}
```

**可用工具**:
- **基础**: `move_forward`, `move_backward`, `turn`, `stop`
- **追击**: `get_enemy_positions`, `chase_enemy` (2个MCP工具)
- **视觉**: `detect_color_and_act`

**追击工具详情**：
1. `get_enemy_positions()` - 获取敌人位置
   - 初始化ROS2订阅器，等待连接建立
   - 刷新回调50次确保接收消息
   - 返回敌人位置JSON字符串

2. `chase_enemy(enemy_positions)` - 追击最近的敌人
   - 接收敌人位置JSON字符串
   - 找到最近的敌人
   - 计算角度并旋转
   - 前进（PID控制，自适应角度校正）
   - 到达后清除敌人

---

### Sim_Module - 2D仿真环境

**作用**: 可视化仿真，执行动作

**输入**: ROS2 `/robot/command`
**输出**: 机器人运动 + 状态发布 `/robot/state`

**组件**：
- `simulator.py` - 主仿真器（含追击功能）
- `enemy_manager.py` - 敌人管理器

**操作**: 鼠标生成敌人 / C清除 / L切换追击线 / ESC退出

**配置**：
- 屏幕尺寸：800x600 像素
- 比例尺：1米 = 100像素
- 帧率：60 FPS
- 敌人位置发布频率：每3秒

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

| 话题 | 方向 | 用途 |
|------|------|------|
| `/robot/command` | Robot → Sim | 动作命令 |
| `/robot/state` | Sim → Robot | 机器人状态 |
| `/robot/enemies` | Sim → Robot | 敌人位置 |
| `/robot/enemy_remove` | Robot → Sim | 清除敌人 |

**消息格式**：
```json
// 动作命令
{"action": "turn", "parameters": {"angle": 54.9, "angular_speed": 0.5}}

// 机器人状态
{"x": 400.0, "y": 300.0, "angle": 0.0}

// 敌人位置
[{"id": "1", "x": 100, "y": 200}, {"id": "2", "x": 500, "y": 400}]
```

---

## 🚀 快速开始

### 安装运行

```bash
# 安装依赖
pip install -r requirements.txt

# 设置 API 密钥
export Test_API_KEY=your_key

# 终端1: 启动仿真器
python3 Sim_Module/sim2d/simulator.py

# 终端2: 启动交互程序
python3 Interactive_Module/interactive.py
```

### 使用示例

```bash
# 基础控制
前进1米 / 左转90度 / 后退0.5米

# 追击功能
追击敌人

# 视觉检测
根据 /path/to/image.png 检测颜色并执行动作

# 复杂任务
先左转45度，前进1.5米，然后右转90度
```

### 追击功能完整流程

1. **启动仿真器**
   ```bash
   python3 Sim_Module/sim2d/simulator.py
   ```
   - 鼠标左键：在点击位置生成敌人
   - 按 C：清除所有敌人
   - 按 L：切换追击线显示
   - 按 ESC：退出仿真器

2. **启动交互程序**
   ```bash
   python3 Interactive_Module/interactive.py
   ```

3. **输入命令**
   ```
   追击敌人
   ```

4. **系统自动执行**
   - 步骤1：获取敌人位置
   - 步骤2：追击最近的敌人
     - 计算角度并旋转
     - 前进（PID控制）
     - 自适应角度校正（如需要）
     - 到达目标（< 5像素）
   - 清除已追击的敌人
   - 可以继续追击下一个敌人

---

## 🎨 追击算法特性

### 角度计算
- 使用 `atan2` 计算目标方向
- 标准化到 [-180°, 180°]
- 自动选择最短旋转路径

### 自适应角度校正
```
检测距离变化:
  距离减小 → 正常前进
  距离增大3次 → 重新计算角度
    - 角度差 > 5° → 重新旋转
    - 角度差 ≤ 5° → 减小步长
```

### PID 控制
- 步长 = 距离 × 0.8
- 最大步长：1.0 米
- 最小步长：0.1 米
- 到达阈值：5 像素

---

## 📝 依赖

```bash
openai>=1.0.0       # LLM API
fastmcp>=0.1.0      # MCP 协议
pygame>=2.5.0       # 2D 仿真
pyyaml>=6.0         # 配置文件
python-dotenv>=1.0.0 # 环境变量
rclpy               # ROS2 Python 客户端
```

---

## 📚 文档

- **CLAUDE.md** - 系统架构和开发指南（适合 Claude Code）
- **process.md** - 数据流、输入输出关系、时序图（详细技术文档）
- **Sim_Module/README.md** - 仿真器详细说明

---

## 🔧 开发

### 添加新工具

1. 在 `Robot_Module/module/` 中创建模块文件
2. 使用 `@mcp.tool()` 装饰器定义工具
3. 在 `register_tools()` 中注册
4. 在 `skill.py` 中调用注册函数

示例：
```python
@mcp.tool()
async def my_tool(param: str) -> str:
    """工具描述

    Args:
        param: 参数说明

    Returns:
        返回值说明
    """
    # 实现代码
    return result
```

### 修改追击参数

编辑 `Robot_Module/module/chase.py`:
```python
MAX_STEP_DISTANCE = 1.0  # 最大步长（米）
ARRIVAL_THRESHOLD = 5.0  # 到达阈值（像素）
```

### 修改仿真器配置

编辑 `Sim_Module/sim2d/simulator.py`:
```python
WIDTH, HEIGHT = 800, 600  # 屏幕尺寸
FPS = 60                  # 帧率
PIXELS_PER_METER = 100    # 比例尺
```

---

## 🐛 常见问题

### Q: 追击时机器人不动？

A: 确保仿真器在运行并且已经生成敌人。查看日志：
- `[Simulator] 发布敌人位置: [...]`
- `[chase.get_enemy_positions] 获取到 N 个敌人`

### Q: 获取不到敌人位置？

A: ROS2 订阅者需要时间建立连接（约1.5秒）。
1. 确保仿真器先启动
2. 等待几秒后再输入"追击敌人"
3. 查看日志中的 `[EnemyPositionsSubscriber] 已更新缓存`

### Q: 机器人在接近目标时来回移动？

A: 已实现自适应角度校正，会自动检测并修正方向。
- 检测距离连续增大3次
- 重新计算角度并旋转
- 如果角度已对准但仍偏离，减小步长

### Q: 追击后第二个敌人还是追到第一个的位置？

A: 已修复。现在追击完成后会：
1. 发送清除命令给仿真器
2. 仿真器立即发布更新后的位置
3. 等待1秒确保消息传递
4. 下次追击获取新位置

---

## 📊 项目结构

```
FinalProject/
├── Interactive_Module/   # 交互界面
├── LLM_Module/           # 双层LLM核心
├── Robot_Module/         # MCP工具注册
│   └── module/
│       ├── base.py       # 基础控制
│       ├── chase.py      # 追击功能
│       └── vision.py     # 视觉检测
├── Sim_Module/           # 2D仿真
│   ├── enemy_manager.py  # 敌人管理
│   └── sim2d/
│       └── simulator.py  # 主仿真器
├── VLM_Module/           # 视觉语言模型
├── Yolo_Module/          # YOLO检测
├── ros_topic_comm.py     # ROS2通信
├── process.md            # 数据流文档
├── CLAUDE.md             # 开发指南
└── README.md             # 本文档
```

---

**模块化，易扩展！** 🚀

详细数据流和架构说明请查看 **process.md**
