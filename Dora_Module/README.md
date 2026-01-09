# Dora_Module - Dora 仿真模块

用于仿真测试的 Dora 模块，支持双层 LLM 架构。

## 🎯 模块概述

本模块提供：
- **Dora 数据流管道** - 基于事件驱动的通信框架
- **仿真器** - 可视化机器人运动
- **双层 LLM 节点** - 任务规划 + 执行
- **UI 输入界面** - 友好的用户交互

## 📁 文件结构

```
Dora_Module/
├── llm_agent_with_mcp.py          # 双层 LLM 节点
├── simulator.py                   # 机器人仿真器
├── input_ui.py                    # UI 输入界面
├── dora-interactive-mcp.yaml      # MCP 数据流配置
├── dora-interactive-v2.yaml       # UI 数据流配置
└── out/                           # 输出目录
```

## 🚀 快速开始

### 1. 安装依赖

```bash
pip install dora-rs pyarrow pyyaml python-dotenv pygame
```

### 2. 配置 API Key

在项目根目录创建 `.env` 文件：

```bash
Test_API_KEY=sk-your-api-key-here
```

获取 API Key: https://dashscope.aliyun.com

### 3. 启动 Dora

```bash
# 启动 Dora 后台服务
dora up

# 运行 MCP 版本（推荐）
dora start dora-interactive-mcp.yaml --attach

# 或运行 UI 版本
dora start dora-interactive-v2.yaml --attach
```

**就这么简单！** 会自动：
- ✅ 启动仿真器窗口
- ✅ 启动 LLM 节点
- ✅ 启动 UI 输入窗口（v2版本）

## 💬 测试指令

### 基础指令
```
前进1米
Turn 90 degrees left
向右转45度
后退50cm
```

### 多步指令（双层 LLM 的优势）
```
先左转90度，再往前走1米
Turn right, go forward 50cm, then turn left
前进1米然后向右转90度再后退半米
```

### 操作指令
```
抓取杯子
Place the book on the table
```

## 🔍 执行流程示例

输入：`先左转90度，再往前走1米`

```
============================================================
🧠 [上层LLM] 任务规划中...
============================================================
✅ [规划完成] 共分解为 2 个子任务
📋 [任务概述] 左转后前进

子任务序列：
  步骤 1: 向左转90度 (转向)
  步骤 2: 向前走1米 (移动)

============================================================
🚀 [开始执行] 按顺序执行子任务
============================================================

【步骤 1/2】
🔧 [工具调用] turn_left({'angle': 90})
📤 [Dora] 发送命令
⏳ [等待] 执行时间: 2.0秒.. ✅ 完成!

【步骤 2/2】
🔧 [工具调用] move_forward({'distance': 1.0, 'unit': 'm'})
📤 [Dora] 发送命令
⏳ [等待] 执行时间: 2.0秒.. ✅ 完成!

============================================================
✅ [执行完成] 任务总结
============================================================
  1. 向左转90度 - ✅ 成功
  2. 向前走1米 - ✅ 成功
```

## 📊 数据流配置

### dora-interactive-mcp.yaml（MCP版本）

```yaml
nodes:
  - id: llm_agent_mcp
    path: llm_agent_with_mcp.py

  - id: simulator
    path: simulator.py

  - id: input_ui
    path: input_ui.py
```

### dora-interactive-v2.yaml（UI版本）

包含 Pygame 可视化窗口和 UI 输入界面。

## 🎨 可视化界面

启动 `dora-interactive-v2.yaml` 后会自动弹出两个窗口：

1. **仿真窗口** - 显示机器人位置和运动
2. **输入窗口** - 输入指令的 UI 界面

在输入窗口中输入指令并按回车即可。

## 🔧 工作原理

```
┌─────────────────────────────────────┐
│  用户输入指令                        │
└──────────┬──────────────────────────┘
           ↓
┌──────────────────────────────────────┐
│  🧠 上层LLM - 任务规划                │
│  将复杂指令分解为子任务                │
└──────────┬──────────────────────────┘
           ↓
    子任务序列 (步骤1, 步骤2, ...)
           ↓
┌──────────────────────────────────────┐
│  ⚙️  下层LLM - 执行子任务              │
│  调用MCP工具                          │
└──────────┬──────────────────────────┘
           ↓
      工具调用序列
           ↓
┌──────────────────────────────────────┐
│  📤 Dora 数据流管道                   │
│  → dora_node.send_output()           │
└──────────┬──────────────────────────┘
           ↓
┌──────────────────────────────────────┐
│  🤖 Simulator 接收命令                │
│  → 更新机器人状态                     │
│  → 可视化显示                         │
└──────────────────────────────────────┘
```

## 🐛 常见问题

### 问题 1: dora: command not found

**解决**:
```bash
pip install dora-rs
```

### 问题 2: ImportError: No module named 'dora'

**解决**:
```bash
pip install dora-rs pyarrow
```

### 问题 3: API Key 未配置

**解决**:
在项目根目录创建 `.env` 文件：
```bash
Test_API_KEY=sk-your-api-key-here
```

### 问题 4: 窗口无法显示

**解决**:
```bash
# 安装 pygame
pip install pygame

# 检查显示环境
echo $DISPLAY
```

## 📊 与 ROS2 版本对比

| 特性 | Dora | ROS2 |
|------|------|------|
| 真实机器人 | ❌ | ✅ |
| 仿真环境 | ✅ | ✅ |
| 可视化 | ✅ Rerun/Pygame | ⚠️  rqt_graph |
| 配置难度 | 简单 | 中等 |
| 启动方式 | 单命令 | 需要多终端 |
| 适用场景 | 开发测试 | 生产部署 |

## 💡 使用建议

1. **开发阶段** - 使用 Dora 版本
   - 快速迭代
   - 可视化调试
   - 无需真实机器人

2. **部署阶段** - 切换到 ROS2 版本
   - 控制真实机器人
   - 生产环境部署

## 📚 相关资源

- [Dora 官方文档](https://dora.cars.ai/)
- [YAML 配置参考](https://dora.cars.ai/guide/concepts/)
- [项目根目录](../README.md)
- [MCP_Server](../MCP_Server/README.md)

---

**提示**: Dora 版本非常适合快速开发和测试，确认无误后再切换到 ROS2 控制真实机器人。
