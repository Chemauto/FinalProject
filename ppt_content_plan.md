# PPT内容规划 - 基于大语言模型的四足机器人自适应交互导航系统

## 第1页：封面

### 标题
基于大语言模型的四足机器人自适应交互导航

### 副标题
**实现与验证**

### 信息
- 报告人：[您的姓名]
- 日期：2025年1月
- 研究方向：机器人智能导航

### 背景
论文来源：
*Adaptive Interactive Navigation of Quadruped Robots using Large Language Models*

---

## 第2页：研究背景与动机

### 传统导航的局限性

```
┌─────────────────────────────────────┐
│  传统导航方式                         │
├─────────────────────────────────────┤
│  ❌ 需要预先构建环境地图               │
│  ❌ 无法处理动态障碍物                │
│  ❌ 非专家用户难以使用                │
│  ❌ 缺乏环境理解与自适应能力           │
└─────────────────────────────────────┘
```

### 我们的解决方案：AINav

**核心思想：** 通过VLM感知环境信息 + LLM进行规划 + 与环境交互实现自适应

**关键优势：**
- ✅ 自然语言交互（非专家友好）
- ✅ 实时视觉感知
- ✅ 自适应重规划
- ✅ 模块化可扩展

---

## 第3页：系统整体架构

### 三层架构设计

```
┌────────────────────────────────────────────────────┐
│              用户交互层 (Interactive)                │
│        自然语言输入 ← → 任务状态反馈                  │
└───────────────────┬────────────────────────────────┘
                    ↓
┌────────────────────────────────────────────────────┐
│            决策规划层 (Decision & Planning)          │
│  ┌──────────────┐      ┌──────────────┐           │
│  │ 上层LLM      │  →   │ 下层LLM      │           │
│  │ 任务规划     │      │ 执行控制     │           │
│  └──────────────┘      └──────────────┘           │
│  ↓ What                  ↓ How                     │
└───────────────────┬────────────────────────────────┘
                    ↓
┌────────────────────────────────────────────────────┐
│         感知与执行层 (Perception & Action)           │
│  ┌──────────┐    ┌──────────────┐                  │
│  │ VLM视觉  │    │ MCP工具库    │                  │
│  │ 感知     │    │ 技能执行     │                  │
│  └──────────┘    └──────────────┘                  │
└───────────────────┬────────────────────────────────┘
                    ↓
              ROS2通信 + 2D仿真
```

### 核心组件对应关系

| 论文组件 | 本项目实现 | 状态 |
|---------|-----------|------|
| LLM Task Planning | 双层LLM架构 | ✅ 已完成 |
| VLM Perception | 本地+远程双VLM | ✅ 已完成 |
| Skill Library | MCP工具注册框架 | ✅ 已完成 |
| RL Motion Planning | 参数化动作执行 | 🔄 仿真验证中 |
| Adaptive Replanning | 动态提示词加载 | ✅ 已完成 |

---

## 第4页：核心创新点1 - 双层LLM任务规划

### 论文方法 vs 本项目实现

#### 论文设计
- **Proposer**: 生成多样化候选方案
- **Evaluator**: 评估方案可行性
- 输入：目标 + 视觉观测
- 输出：obj-level → skill-level 任务分解

#### 本项目实现（已落地）

**上层LLM - 任务规划器**
```
输入: "前进1米然后左转90度，再根据图片检测颜色"
  ↓
分解: [
  {step:1, task:"前进1米", type:"移动"},
  {step:2, task:"左转90度", type:"转向"},
  {step:3, task:"检测颜色并执行", type:"视觉"}
]
```

**下层LLM - 执行控制器**
```
输入: "前进1米"
  ↓
工具调用: move_forward(distance=1.0, speed=0.3)
  ↓
动作执行: ROS2 Topic → 仿真器
```

### 技术实现

```python
# 核心代码框架
class LLMAgent:
    def plan_tasks(self, user_input, tools):
        # 上层LLM：任务分解
        return tasks_sequence

    def execute_single_task(self, task, tools):
        # 下层LLM：工具调用
        return tool_call_result

    def run_pipeline(self, user_input, tools):
        # 完整流程
        tasks = self.plan_tasks(user_input, tools)
        for task in tasks:
            self.execute_single_task(task, tools)
```

**优势：**
- 规划与执行解耦，提高成功率
- 支持复杂多步任务
- 易于调试和优化

---

## 第5页：核心创新点2 - VLM视觉感知

### 视觉感知架构

```
图像输入 (PNG/JPG)
    ↓
┌─────────────────────────┐
│     VLM模型选择           │
├─────────────────────────┤
│ • 本地: Ollama+Qwen3-VL  │
│ • 远程: 通义千问VL API   │
└─────────────────────────┘
    ↓
颜色识别 + 语义理解
    ↓
动作映射策略
```

### 颜色-动作映射表

| 视觉输入 | 识别结果 | 动作输出 | 应用场景 |
|---------|---------|---------|---------|
| 🟥 红色方块 | red | 前进1米 | 通行信号 |
| 🟨 黄色方块 | yellow | 左转90° | 左转指引 |
| 🟩 绿色方块 | green | 后退1米 | 避障调整 |
| 🟦 蓝色方块 | blue | 右转90° | 右转指引 |
| 🟪 紫色方块 | purple | 紧急停止 | 终止信号 |

### 论文对应

**论文方法：**
- RGB相机 + LiDAR输入
- VLM生成语义描述
- LLM根据观测调整规划

**本项目实现：**
- 单目图像输入
- VLM直接识别颜色标志
- 自动映射到预定义动作

**扩展能力：**
- 支持多种VLM后端（可切换）
- 异步API调用，不阻塞主流程
- 可扩展到更多视觉特征检测

---

## 第6页：核心创新点3 - MCP模块化工具库

### 论文 Skill Library 设计

**分层技能体系：**
```
Low-level Skills (RL预训练)
├── Walking (行走)
├── Climbing up/down (上下楼梯)
└── High-level Skills
    ├── Navigation skill (导航)
    └── Pushing skill (推物)
```

### 本项目 MCP 工具框架（已实现）

```
Robot_Module/
├── skill.py (FastMCP服务器)
└── module/
    ├── base.py (底盘控制)
    │   ├── move_forward()
    │   ├── move_backward()
    │   ├── turn()
    │   └── stop()
    └── vision.py (视觉感知)
        └── detect_color_and_act()
```

### 工具注册流程

```python
# 1. 定义工具（带完整文档）
@mcp.tool()
async def move_forward(distance: float, speed: float) -> str:
    """向前移动指定距离

    Args:
        distance: 移动距离（米）
        speed: 移动速度（米/秒）

    Returns:
        动作指令JSON
    """
    action = {'action': 'move_forward',
              'parameters': {'distance': distance, 'speed': speed}}
    _get_action_queue().put(action)
    return json.dumps(action)

# 2. 注册到MCP服务器
def register_tools(mcp):
    return {'move_forward': move_forward}
```

### 关键特性

✅ **懒加载初始化** - 按需创建ROS2/VLM连接
✅ **零配置扩展** - 新工具只需添加装饰器和注册
✅ **类型安全** - 完整参数类型和文档
✅ **异步执行** - 支持并发工具调用

---

## 第7页：核心创新点4 - 自适应重规划机制

### 论文方法：Advisor + Arborist

**Advisor（顾问）：**
- 监测环境变化
- 判断是否需要重规划
- 修改执行指令

**Arborist（园艺师）：**
- 维护技能树
- 动态剪枝/分支
- 选择最优执行路径

### 本项目实现：动态提示词加载

**实现机制：**

```python
# 运行时动态生成系统提示
def load_dynamic_prompt(tools):
    robot_config = format_robot_config(tools)  # 当前能力
    available_skills = format_available_skills(tools)  # 可用技能

    prompt = template.format(
        robot_config=robot_config,
        available_skills=available_skills
    )
    return prompt
```

**自适应场景示例：**

| 场景 | 触发条件 | 自适应响应 |
|-----|---------|-----------|
| 新工具添加 | MCP注册新技能 | 自动更新规划LLM的知识库 |
| 参数调整 | 用户修改速度/角度 | 实时更新约束条件 |
| 视觉反馈 | VLM检测到新颜色 | 动态调整下一步动作 |
| 任务失败 | 工具执行异常 | 继续执行后续任务（容错） |

### 对比优势

| 特性 | 论文方法 | 本项目实现 |
|-----|---------|-----------|
| 重规划触发 | Advisor判断 | 每个子任务独立规划 |
| 技能树维护 | Arborist动态剪枝 | MCP工具库懒加载 |
| 响应速度 | 需要维护树结构 | 即时加载当前能力 |
| 实现复杂度 | 高 | 低（模板化） |

---

## 第8页：系统实现 - ROS2通信架构

### 通信流程图

```
┌────────────────┐
│ Robot_Module   │
│   (Publisher)  │
└────────┬───────┘
         │ put(action)
         ↓
    ROS2 Topic
    /robot/command
    (String/JSON)
         ↓
┌────────────────┐
│ Sim_Module     │
│  (Subscriber)  │
└────────┬───────┘
         │ callback(action)
         ↓
    动作执行 + 可视化
```

### 消息格式

**发布消息：**
```json
{
  "action": "move_forward",
  "parameters": {
    "distance": 1.0,
    "speed": 0.3
  }
}
```

**订阅处理：**
```python
def execute_action(action):
    action_type = action.get('action')
    params = action.get('parameters')

    if action_type == 'move_forward':
        distance = params['distance']
        speed = params['speed']
        # 执行移动逻辑
        target_position = calculate_new_pos(distance, speed)
```

### 关键实现

✅ **线程安全队列** - queue.Queue() 缓冲
✅ **非阻塞处理** - spin_once(timeout=0.001)
✅ **自动初始化** - 懒加载避免启动依赖
✅ **跨平台兼容** - Windows/Linux支持

---

## 第9页：仿真环境与实验验证

### 本项目仿真环境

**2D Pygame仿真器**

```
┌─────────────────────────────────┐
│           2D Simulator            │
├─────────────────────────────────┤
│  • 机器人：圆形底盘 + 方向指示     │
│  • 环境：网格背景 + 边界限制       │
│  • 状态：位置(x,y) + 角度(θ)      │
│  • 可视化：实时运动轨迹           │
└─────────────────────────────────┘
```

### 功能对比

| 功能 | 论文(Isaac Lab) | 本项目(Pygame) |
|-----|----------------|---------------|
| 3D仿真 | ✅ | 🔄 2D实现 |
| 四足机器人 | ✅ | 🔄 差速驱动模型 |
| 物理引擎 | PhysX | 简化运动学 |
| 障碍物 | ✅ | 🔄 待添加 |
| 多任务场景 | ✅ | 🔄 扩展中 |
| 实时性 | 中等 | 高（60 FPS） |

### 已验证场景

✅ **基础运动：** 前进、后退、旋转
✅ **视觉交互：** 颜色检测 → 动作映射
✅ **复合任务：** 多步骤自然语言指令
✅ **异常处理：** 工具失败容错

---

## 第10页：实验演示

### 示例1：基础导航任务

**用户指令：**
```
"前进1米然后左转90度"
```

**系统执行流程：**
```
[上层LLM] 规划任务序列:
  └─ 步骤1: 前进1米
  └─ 步骤2: 左转90度

[下层LLM] 执行步骤1:
  ├─ 工具调用: move_forward(distance=1.0, speed=0.3)
  ├─ ROS2发布: {"action":"move_forward",...}
  └─ 仿真器: 机器人从(400,300)移动到(400,200)

[下层LLM] 执行步骤2:
  ├─ 工具调用: turn(angle=90.0, angular_speed=0.5)
  ├─ ROS2发布: {"action":"turn",...}
  └─ 仿真器: 机器人从0°旋转到90°

✅ 任务完成: 2/2 成功
```

### 示例2：视觉引导任务

**用户指令：**
```
"根据 /path/to/red.png 检测颜色并执行动作"
```

**系统执行流程：**
```
[VLM] 分析图像:
  ├─ 输入: red.png
  ├─ 模型: Qwen-VL (本地/远程)
  ├─ 识别: {"color": "red"}
  └─ 映射: action = "move_forward"

[下层LLM] 执行动作:
  ├─ 工具调用: detect_color_and_act(image_path)
  ├─ 内部逻辑: 红色 → 前进1米
  └─ 仿真器: 执行移动

✅ 任务完成: 视觉引导成功
```

### 示例3：复杂复合任务

**用户指令：**
```
"前进1米，然后根据图片检测颜色，最后右转45度"
```

**执行结果：**
```
步骤1: ✅ 前进1米
步骤2: ✅ 检测到黄色 → 左转90度
步骤3: ✅ 右转45度
总体: ✅ 3/3 任务完成
```

---

## 第11页：总结与展望

### 项目总结

#### 已完成功能 ✅

| 模块 | 功能 | 完成度 |
|-----|------|--------|
| LLM_Module | 双层LLM规划+执行 | ✅ 100% |
| VLM_Module | 本地+远程双VLM | ✅ 100% |
| Robot_Module | MCP工具注册框架 | ✅ 100% |
| Interactive_Module | CLI交互界面 | ✅ 100% |
| Sim_Module | 2D仿真环境 | ✅ 100% |
| ROS2通信 | Topic发布/订阅 | ✅ 100% |

#### 技术亮点 🌟

1. **双层LLM解耦** - 规划与执行分离，提高成功率
2. **动态提示词** - 运行时加载系统能力，无需硬编码
3. **懒加载架构** - 按需初始化，降低启动依赖
4. **多VLM支持** - 本地/远程灵活切换
5. **模块化设计** - MCP工具库易于扩展

#### 与论文对标

| 论文组件 | 实现方式 | 完成度 |
|---------|---------|--------|
| LLM Task Planning | 双层LLM + 动态提示词 | ✅ |
| VLM Perception | Ollama + API双方案 | ✅ |
| Skill Library | MCP工具注册框架 | ✅ |
| Motion Planning | 参数化动作执行 | 🔄 仿真验证 |
| Adaptive Replanning | 子任务级重规划 | ✅ |
| Quadruped Robot | 2D仿真模型 | 🔄 待实物部署 |

### 未来展望 🔮

#### 短期目标（1-2个月）

- [ ] 添加障碍物检测与避障
- [ ] 实现更多VLM感知场景
- [ ] 完善仿真环境物理引擎
- [ ] 添加Web可视化界面

#### 中期目标（3-6个月）

- [ ] 迁移到3D仿真环境（Isaac Lab）
- [ ] 集成RL预训练的运动策略
- [ ] 部署到真实四足机器人平台
- [ ] 实现复杂多物体交互任务

#### 长期愿景

- [ ] 多机器人协同导航
- [ ] 强化学习优化执行策略
- [ ] 开源工具库，支持更多机器人平台

---

## 附录：技术栈

```
┌─────────────────────────────────────┐
│         应用技术栈                    │
├─────────────────────────────────────┤
│ 语言: Python 3.10+                  │
│ LLM:  Qwen3-32B (通义千问)          │
│ VLM:  Qwen-VL / Ollama              │
│ MCP:  FastMCP框架                   │
│ 通信: ROS2 Humble                   │
│ 仿真: Pygame 2D                     │
│ 环境: Windows/Linux                 │
└─────────────────────────────────────┘
```

---

## 演示Demo（可选）

### 视频展示 / 现场演示

1. **基础导航** - 前进、后退、旋转
2. **视觉交互** - 颜色检测与动作
3. **复合任务** - 多步骤自然语言控制
4. **实时可视化** - 2D仿真器界面

---

**感谢聆听！欢迎提问与交流**
