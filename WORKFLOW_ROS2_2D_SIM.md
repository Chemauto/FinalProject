# ROS2 (2D 仿真) 工作流程详解

当您在 ROS2 (2D 仿真) 模式下输入"向前走1米"后,整个系统从接收指令到仿真器动作的完整工作流程如下。

### 1. 启动与初始化

- 运行 `./start_ros2_mcp.sh --sim 2d` 脚本会启动两个独立的Python进程：
    1.  **2D仿真器**: `Sim_Module/ros2_2d/simulator.py`
    2.  **主交互程序**: `Middle_Module/ROS/ros2_interactive_mcp.py`
- 仿真器进程会创建一个窗口并订阅ROS2的 `/cmd_vel` 话题,等待指令。
- 主交互程序会加载所有需要的模块, 包括 `LLM_Module`, `MCP_Module` (它会自动扫描并注册 `Robot_Module/Sim_2D/skills` 里的所有技能), 并准备接收您的输入。

### 2. 用户输入

- 您在 `ros2_interactive_mcp.py` 程序的终端里输入 "向前走1米"。

### 3. LLM处理 (双层结构)

- **上层LLM (任务规划)**: `LLM_Module`接收到指令。对于 "向前走1米" 这种简单指令, 它会判断这本身就是一个独立的子任务。
- **下层LLM (执行控制)**: `LLM_Module` 将这个子任务转换为一个具体的、可执行的函数调用。它会查看从 `MCP_Module` 获取的可用技能列表, 发现 `sim_2d_skills.py` 中的 `move(direction: str, distance: float)` 函数最匹配, 于是生成函数调用 `move(direction='forward', distance=1.0)`。

### 4. MCP技能调用

- `MCP_Module` (技能中间件) 接收到 `move(direction='forward', distance=1.0)` 的调用请求。
- 它从技能注册表中找到对应的函数, 执行 `Robot_Module/Sim_2D/skills/sim_2d_skills.py` 文件里的 `move` 函数。
- `move` 函数执行后, 并不直接控制机器人, 而是返回一个标准化的指令字典, 内容为: `{'action': 'move', 'parameters': {'direction': 'forward', 'distance': 1.0}}`。

### 5. ROS2桥接与通信

- `Middle_Module/ROS/ros2_robot_controller.py` (通用ROS2桥接) 接收到这个指令字典。
- 它根据 `action` 的值 (`'move'`) 和 `parameters`, 将其转换成一个标准的ROS2消息 (`Twist` 类型)。对于向前移动1米, 它会设置消息的 `linear.x` 字段为一个正值 (代表速度), 其他字段为0。
- 然后, `ros2_robot_controller` 将这个 `Twist` 消息发布到 `/cmd_vel` 话题上。

### 6. 仿真器执行

- 运行在另一个进程中的 `Sim_Module/ros2_2d/simulator.py` 一直在监听 `/cmd_vel` 话题。
- 它收到了刚刚发布的速度指令消息。
- 根据消息内容, 仿真器开始在窗口中更新机器人的位置,使其向前移动,直到达到指令要求的效果。

### 函数调用关系图 (Sequence Diagram)

```mermaid
sequenceDiagram
    participant User
    participant A as ros2_interactive_mcp.py (主交互)
    participant B as LLM_Module (大模型)
    participant C as MCP_Module (技能中间件)
    participant D as sim_2d_skills.py (机器人技能)
    participant E as ros2_robot_controller.py (ROS2桥接)
    participant F as simulator.py (2D仿真器)

    User->>+A: 输入 "向前走1米"
    A->>+B: agent.run_pipeline("向前走1米")
    B->>B: 任务规划 -> "向前走1米"
    B->>B: 技能选择 -> move(direction='forward', distance=1.0)
    B->>+C: mcp_bridge.execute_skill("move", ... )
    C->>+D: move(direction='forward', distance=1.0)
    D-->>-C: return {'action': 'move', 'parameters': ...}
    C-->>-B: 返回技能执行结果(指令字典)
    B-->>-A: 返回结果
    A->>+E: handle_action(result)
    E->>E: 将指令字典转换为ROS2 Twist消息
    E-->>-F: 发布消息到 /cmd_vel 话题
    deactivate E
    F->>F: 接收 /cmd_vel 消息, 更新机器人位置
    F->>F: 重新绘制UI
```
