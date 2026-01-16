# Interactive_Module - 交互界面

## 概述

Interactive_Module 是系统的**用户交互入口**，负责接收用户自然语言指令，协调 LLM_Module 和 Robot_Module 完成任务执行，并显示结果。

### 核心功能

- **交互界面**: 提供命令行交互界面 (CLI)
- **模块协调**: 连接 LLM_Module 和 Robot_Module
- **动态提示词**: 自动生成包含可用技能列表的提示词
- **结果展示**: 格式化显示任务规划和执行结果
- **自动初始化**: ROS2 通讯层自动管理，无需手动配置

## 文件结构

```
Interactive_Module/
├── interactive.py        # 交互界面主程序
└── README.md
```

## 数据流

```
用户输入
    ↓
┌─────────────────────────────────────────┐
│ Interactive_Module.interactive.py      │
│                                         │
│ 1. 接收用户自然语言指令                  │
│ 2. 注册 Robot_Module 工具（自动）       │
│ 3. 动态加载并填充提示词                  │
│ 4. 调用 LLM_Module 双层LLM               │
│ 5. 执行 MCP 工具函数                     │
│ 6. 显示执行结果                          │
└──────────────┬──────────────────────────┘
               ↓
        ┌──────┴──────┐
        ↓             ↓
  LLM_Module    Robot_Module
  (规划+执行)    (工具调用 + ROS2发布)
        ↓             ↓
    ROS2 Topic (/robot/command)
        ↓
  Sim_Module (仿真执行)
```

## 核心文件: interactive.py

### 主函数流程

```python
def main():
    """主函数"""
    # 1. 注册所有 Robot_Module 的工具函数
    register_all_modules()

    # 2. 检查 API Key
    api_key = os.getenv('Test_API_KEY')

    # 3. 从 Robot_Module 获取工具定义
    tools = get_tool_definitions()

    # 4. 获取提示词路径
    prompt_path = "LLM_Module/prompts/planning_prompt_2d.yaml"

    # 5. 初始化 LLM Agent
    llm_agent = LLMAgent(api_key=api_key, prompt_path=str(prompt_path))

    # 6. 动态加载并填充提示词
    dynamic_prompt = load_dynamic_prompt(prompt_path, tools)
    llm_agent.planning_prompt_template = dynamic_prompt

    # 7. 主循环 - 接收用户输入
    while True:
        user_input = input("💬 请输入指令: ").strip()

        # 8. 执行双层 LLM 流程
        results = llm_agent.run_pipeline(
            user_input=user_input,
            tools=tools,
            execute_tool_fn=execute_tool
        )

        # 9. 显示结果
        if results:
            success_count = sum(1 for r in results if r.get("success"))
            print(f"📊 [完成] {success_count}/{len(results)} 个任务成功")
```

### 工具执行函数

```python
def execute_tool(function_name: str, function_args: dict) -> dict:
    """执行 Robot_Module 中的工具函数

    由 LLM_Module 的下层 LLM 调用

    Args:
        function_name: 工具函数名 (如 "move_forward")
        function_args: 函数参数 (如 {"distance": 1.0, "speed": 0.3})

    Returns:
        执行结果字典
    """
    # 1. 获取工具函数
    skill_func = get_skill_function(function_name)

    if not skill_func:
        return {"error": f"Unknown tool: {function_name}"}

    # 2. 调用异步技能函数（自动初始化 ROS2 队列）
    result = asyncio.run(skill_func(**function_args))

    # 3. 估算执行时间
    delay = calculate_execution_delay(function_name, function_args)

    return {"result": result, "delay": delay}
```

### 动态提示词加载

```python
def load_dynamic_prompt(prompt_path, tools):
    """加载并动态填充提示词

    将可用技能列表插入到提示词模板中

    Args:
        prompt_path: 提示词 YAML 文件路径
        tools: 工具定义列表

    Returns:
        填充后的提示词字符串
    """
    import yaml

    # 1. 加载 YAML 模板
    with open(prompt_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    # 2. 动态生成配置信息
    robot_config = format_robot_config(tools)
    available_skills = format_available_skills(tools)

    # 3. 填充模板
    prompt = data.get("prompt", "").format(
        robot_config=robot_config,
        available_skills=available_skills,
        user_input="{user_input}"  # 保留占位符供后续填充
    )

    return prompt
```

## ROS2 通讯

### 自动初始化

ROS2 队列在工具函数首次调用时**自动初始化**，无需手动配置：

```python
# Robot_Module/module/base.py
def _get_action_queue():
    """获取动作队列（懒加载：首次使用时自动初始化 ROS 队列）"""
    global _action_queue
    if _action_queue is None:
        from ros_topic_comm import get_shared_queue
        _action_queue = get_shared_queue()  # 自动创建 Publisher
    return _action_queue
```

### 话题发布

工具函数调用时自动发布到 ROS2 话题：

```python
async def move_forward(distance: float = 1.0, speed: float = 0.3) -> str:
    action = {'action': 'move_forward', 'parameters': {...}}
    _get_action_queue().put(action)  # 发布到 /robot/command
    return json.dumps(action)
```

## 使用示例

### 启动交互界面

```bash
# 方法1: 使用启动脚本
./start_robot_system.sh

# 方法2: 直接运行
python3 Interactive_Module/interactive.py
```

### 交互示例

```
============================================================
LLM Interactive Interface
============================================================
API: https://dashscope.aliyuncs.com/compatible-mode/v1/
Model: qwen-plus
可用工具: 4 个
------------------------------------------------------------
  • move_forward(distance: number, speed: number)
    描述: 向前移动指定距离

  • move_backward(distance: number, speed: number)
    描述: 向后移动指定距离

  • turn(angle: number, angular_speed: number)
    描述: 旋转指定角度

  • stop
    描述: 紧急停止机器人
------------------------------------------------------------
提示: 确保已在另一个窗口启动仿真器
输入 'quit' 或 'exit' 退出
============================================================

💬 请输入指令: 前进1米然后左转90度

████████████████████████████████████████████████████████████
📥 [用户输入] 前进1米然后左转90度
████████████████████████████████████████████████████████████

============================================================
🧠 [上层LLM] 任务规划中...
============================================================
✅ [规划完成] 共分解为 2 个子任务
📋 [任务概述] 用户需要先向前移动1米，然后向左旋转90度

子任务序列：
  步骤 1: 向前移动1米 (移动)
  步骤 2: 左转90度 (旋转)

////////////////////////////////////////////////////////////
🚀 [开始执行] 按顺序执行子任务
////////////////////////////////////////////////////////////

【步骤 1/2】
──────────────────────────────────────────────────
⚙️  [执行中] 向前移动1米
──────────────────────────────────────────────────
🔧 [工具调用] move_forward({'distance': 1.0, 'speed': 0.3})
[base.py] ROS队列已自动初始化
[ros_topic_comm] ROS 已初始化
[ActionPublisher] ROS话题发布器已创建: /robot/command
[ActionPublisher] 发布命令: {'action': 'move_forward', 'parameters': {'distance': 1.0, 'speed': 0.3}}
⏳ [等待] 执行时间: 3.3秒... ✅ 完成!

【步骤 2/2】
──────────────────────────────────────────────────
⚙️  [执行中] 左转90度
──────────────────────────────────────────────────
🔧 [工具调用] turn({'angle': 90.0, 'angular_speed': 0.5})
[ActionPublisher] 发布命令: {'action': 'turn', 'parameters': {'angle': 90.0, 'angular_speed': 0.5}}
⏳ [等待] 执行时间: 3.1秒... ✅ 完成!

////////////////////////////////////////////////////////////
✅ [执行完成] 任务总结
////////////////////////////////////////////////////////////
  1. 向前移动1米 - ✅ 成功
  2. 左转90度 - ✅ 成功

📊 [完成] 2/2 个任务成功

💬 请输入指令:
```

## 启动时的输出

### 1. 工具注册信息

```
============================================================
[skill.py] 开始注册机器人技能模块...
============================================================
[base.py:register_tools] 底盘控制模块已注册 (4 个工具)
============================================================
[skill.py] ✓ 所有模块注册完成
============================================================
```

### 2. ROS2 初始化

```
[base.py] ROS队列已自动初始化
[ros_topic_comm] ROS 已初始化
[ActionPublisher] ROS话题发布器已创建: /robot/command
```

### 3. 欢迎界面

```
============================================================
LLM Interactive Interface
============================================================
API: https://dashscope.aliyuncs.com/compatible-mode/v1/
Model: qwen-plus
可用工具: 4 个
------------------------------------------------------------
  • move_forward(distance: number, speed: number)
    参数: distance, speed
    描述: 向前移动指定距离

  • move_backward(distance: number, speed: number)
    参数: distance, speed
    描述: 向后移动指定距离

  • turn(angle: number, angular_speed: number)
    参数: angle, angular_speed
    描述: 旋转指定角度

  • stop
    描述: 紧急停止机器人
------------------------------------------------------------
```

## 退出系统

```
💬 请输入指令: quit
👋 再见!
```

或使用 `Ctrl+C`:

```
^C
👋 再见!
[skill.py] 清理资源完成
```

## 错误处理

### API Key 未设置

```
❌ 错误: 未设置 Test_API_KEY 环境变量
请设置: export Test_API_KEY=your_api_key_here
```

### 仿真器未启动

```
提示: 确保已在另一个窗口启动仿真器
  python3 Sim_Module/sim2d/simulator.py
```

### 无效指令

```
💬 请输入指令: 跳舞
[上层LLM] 规划失败: 无法识别的指令
[回退] 将作为单个任务处理
[下层LLM] 执行中: 跳舞
⚠️ [警告] 未找到合适的工具
```

## 依赖

```
openai>=1.0.0         # OpenAI API 客户端
pyyaml>=6.0           # YAML 配置解析
python-dotenv>=1.0.0  # 环境变量管理
rclpy                 # ROS2 Python 客户端库（自动导入）
```

## 环境变量

```bash
# 必需
Test_API_KEY=your_api_key_here

# 可选
ROBOT_MODEL_TYPE=2d   # 机器人类型 (2d/go2)
```

## 设计特点

1. **模块解耦**: 不包含具体的 LLM 逻辑，只负责协调
2. **动态提示词**: 根据可用工具自动生成提示词
3. **自动初始化**: ROS2 通讯层自动管理，零配置
4. **清晰输出**: 格式化显示执行过程
5. **错误恢复**: 完善的错误处理机制
6. **易于调试**: 详细的日志输出

## 与其他模块的关系

```
┌─────────────────────────────────────────┐
│     Interactive_Module                 │
│     (交互界面 - 入口)                    │
└──────────────┬──────────────────────────┘
               │
       ┌───────┴───────┐
       ↓               ↓
┌──────────────┐  ┌──────────────┐
│ LLM_Module   │  │ Robot_Module │
│ (规划+执行)   │  │ (工具注册)    │
└──────────────┘  └──────────────┘
       ↓               ↓
   MCP 工具调用    ROS2 发布
       ↓               ↓
   └───────────────┘
          ↓
    ROS2 Topic
       ↓
  ┌──────────────┐
  │ Sim_Module   │
  │ (仿真执行)    │
  └──────────────┘
```

## 相关文档

- [主项目 README](../README.md)
- [LLM_Module README](../LLM_Module/README.md)
- [Robot_Module README](../Robot_Module/README.md)
- [Sim_Module README](../Sim_Module/README.md)
- [ros_topic_comm.py](../ros_topic_comm.py) - ROS2 通讯模块

---

**用户友好，功能强大！** 🚀
