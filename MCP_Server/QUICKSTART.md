# MCP Robot Control - 快速入门

## 🚀 5分钟快速开始

### 方式1: 使用Dora仿真器（推荐用于测试）

```bash
# 1. 安装依赖
pip install mcp dora-rs pyarrow pyyaml python-dotenv

# 2. 启动Dora
cd Dora_Module
dora up

# 3. 运行MCP版本
dora start dora-interactive-mcp.yaml --attach
```

### 方式2: 使用ROS1控制真实机器人

```bash
# 1. 安装ROS1 (如果还没有)
# 参考: http://wiki.ros.org/ROS/Installation

# 2. 启动ROS1 master
roscore

# 3. 创建并启动ROS1订阅节点
# 使用 MCP_Server/README.md 中的示例代码

# 4. 使用MCP (方式A或B)
# 方式A: 作为独立服务器
python MCP_Server/mcp_robot_server.py --adapter ros1

# 方式B: 集成到LLM应用
# 在你的代码中导入并使用
```

## 💬 测试指令

### 基础指令
- `前进1米`
- `Turn 90 degrees left`
- `向右转45度`

### 多步指令（MCP的强大之处）
- `先左转90度，再往前走1米`
- `Turn right, go forward 50cm, then turn left`
- `前进1米然后向右转90度再后退半米`

### 操作指令
- `抓取杯子`
- `Place the book on the table`

## 🔍 工作原理

```
用户: "先左转90度，再往前走1米"
  ↓
LLM (qwen-plus + MCP Tools)
  ↓
自动解析并调用:
  - turn_left(90)
  - move_forward(1.0, "m")
  ↓
Adapter (Dora/ROS1)
  ↓
执行动作
```

## 📊 与旧版本对比

| 你的指令 | 旧版本结果 | MCP版本结果 |
|---------|-----------|------------|
| `先左转90度，再往前走1米` | ❌ 只左转 | ✅ 左转+前进 |
| `前进1米` | ✅ 前进1米 | ✅ 前进1米 |
| `Turn left, then go forward` | ❌ 只转左 | ✅ 转左+前进 |

## 🎯 核心优势

1. **多步骤指令** - 完美支持复杂的指令序列
2. **框架无关** - 同一套代码支持Dora和ROS1
3. **真实机器人** - 通过ROS1控制实际机器人
4. **易于扩展** - 添加新skill只需定义函数
5. **类型安全** - 自动参数验证

## 🔧 配置

编辑 `MCP_Server/config.yaml`:

```yaml
adapter:
  type: "dora"  # 或 "ros1"

  ros1:
    node_name: "mcp_robot_control"
    topic_name: "/robot_command"
```

## 📚 下一步

- 阅读完整文档: `MCP_Server/README.md`
- 查看ROS1集成示例
- 添加自定义robot skills

## 🐛 遇到问题?

1. **Dora适配器不可用**: `pip install dora-rs`
2. **ROS1适配器不可用**: 先安装ROS1
3. **多步指令不执行**: 确保使用MCP版本（`dora-interactive-mcp.yaml`）
