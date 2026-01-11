# Dora MCP 系统调试总结

## 概述
本文档记录了在调试 Dora + MCP + 双层LLM 系统时遇到的所有问题及解决方案。

---

## 问题 1: 缺失的 Python 文件

### 错误信息
```
/home/robot/miniconda3/envs/qwen3/bin/python3: can't open file '/home/robot/work/FinalProject/Middle_Module/Dora/simulator.py': [Errno 2] No such file or directory
/home/robot/miniconda3/envs/qwen3/bin/python3: can't open file '/home/robot/work/FinalProject/Middle_Module/Dora/llm_agent_with_mcp.py': [Errno 2] No such file or directory
```

### 原因
在项目重构过程中，仿真器文件被移动到了 `Sim_Module/dora_2d/`，但：
1. YAML 配置文件仍引用旧路径
2. `llm_agent_with_mcp.py` 文件不存在

### 解决方案
1. 更新 `dora-interactive-mcp.yaml`:
   ```yaml
   - id: simulator
     path: ../../Sim_Module/dora_2d/simulator.py
   ```

2. 创建 `llm_agent_with_mcp.py` (参考 ROS2 版本的 `ros2_interactive_mcp.py`)

---

## 问题 2: LLM 不思考，没有任务规划

### 现象
UI 发送了命令，但 `llm-agent-mcp` 节点没有输出任何接收信息。

### 原因
Dora 事件循环使用方式不正确：
- 错误: `for event in node:` 并检查 `event["type"] == "input"` (小写)
- 正确: `node.next()` 并检查 `event["type"] == "INPUT"` (大写)

### 解决方案
修改事件循环：
```python
# 错误的方式
for event in node:
    if event["type"] == "input":
        ...

# 正确的方式
while True:
    event = node.next()
    if event is None:
        continue
    if event["type"] == "INPUT":
        ...
```

---

## 问题 3: 仿真器没有变化

### 现象
任务规划正常，LLM 正确调用了技能，但仿真器中的机器人没有移动或转向。

### 原因
1. **仿真器只处理 `action == "navigate"`**
   - 技能发送的是 `action: "turn"`
   - 仿真器代码: `if action == "navigate":`

2. **数据格式错误**
   - 错误: `node.send_output("command", json.dumps(dora_output).encode())`
   - 正确: `node.send_output("command", pa.array([dora_output]))`

### 解决方案
1. 扩展仿真器支持的 action 类型:
   ```python
   if action in ["navigate", "turn", "turn_left", "turn_right"]:
       robot.set_navigation_goal(parameters)
   ```

2. 使用 PyArrow 数组发送数据:
   ```python
   import pyarrow as pa
   node.send_output("command", pa.array([dora_output]))
   ```

---

## 问题 4: 机器人左右转反了

### 现象
- 发送"左转90度"，机器人向右转
- 发送"右转45度"，机器人向左转

### 原因
仿真器中的角度计算错误：
- 错误: `self.target_angle = (self.angle - angle_value) % 360`
- 这导致正角度向右转（顺时针）

### 解决方案
修改为加法：
```python
self.target_angle = (self.angle + angle_value) % 360
```

---

## 问题 5: 无法正确退出

### 现象
按 Ctrl+C 后，进程没有正确退出，需要手动 kill。

### 原因
没有处理 Dora 的 `STOP` 事件。

### 解决方案
在事件循环中添加 STOP 处理：
```python
if event["type"] == "STOP":
    print("🛑 收到 STOP 事件，正在退出...")
    break
```

---

## 关键要点总结

### 1. Dora 事件处理
- 使用 `node.next()` 获取事件
- 事件类型是 `"INPUT"` (大写)，不是 `"input"` (小写)
- 使用 `event["id"]` 获取输入 ID
- 使用 `event["value"][0].as_py()` 获取数据

### 2. Dora 数据发送
- 使用 PyArrow 数组: `pa.array([data])`
- 不要用 JSON 字符串编码

### 3. YAML 配置
- 路径是相对于 YAML 文件所在目录的
- 使用 `../../` 向上两级目录

### 4. 角度约定
- 正角度 = 左转（逆时针）
- 负角度 = 右转（顺时针）
- Pygame 中 `angle + value` 是逆时针（因为 Y 轴向下）

---

## 修改的文件列表

1. `/home/robot/work/FinalProject/Middle_Module/Dora/dora-interactive-mcp.yaml`
   - 更新仿真器路径

2. `/home/robot/work/FinalProject/Middle_Module/Dora/llm_agent_with_mcp.py`
   - 新建文件
   - 正确处理 Dora 事件循环
   - 正确发送 PyArrow 数据

3. `/home/robot/work/FinalProject/Sim_Module/dora_2d/simulator.py`
   - 扩展支持的 action 类型
   - 添加 STOP 事件处理
   - 修复角度计算（左/右转）

---

## 调试技巧

1. **添加调试输出**
   - 在事件循环开始时打印每个事件
   - 在发送数据前打印数据内容

2. **逐步验证**
   - 先验证事件是否被接收
   - 再验证数据是否被发送
   - 最后验证仿真器是否处理

3. **参考工作版本**
   - ROS2 版本是正常工作的
   - 对比 ROS2 和 Dora 的差异

---

## 后续改进建议

1. **统一数据格式**
   - 考虑在 MCP 层统一处理数据格式转换
   - 避免每个适配器重复处理

2. **错误处理**
   - 添加更详细的错误信息
   - 验证数据格式

3. **文档**
   - 为 Dora 适配器编写专门文档
   - 记录数据格式规范
