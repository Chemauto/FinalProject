# 🤖 Robot Control with MCP + LLM

基于 Model Context Protocol (MCP) 的机器人控制系统，支持双层 LLM 架构（任务规划 + 执行）。

## 🎯 项目特点

- **双层 LLM 架构** - 上层任务规划，下层任务执行
- **多模态支持** - 支持 ROS2、Dora 等多种通信方式
- **自然语言交互** - 支持中英文复杂指令
- **易于扩展** - 模块化设计，可轻松添加新技能

## 📁 项目结构

```
FinalProject/
├── MCP_Server/      # 核心：MCP 服务器 + 技能库
├── ROS_Module/      # ROS2 通信模块（真实机器人）
├── Dora_Module/     # Dora 仿真模块（仿真测试）
└── VLM_Modele/      # VLM Demo 程序
```

## 🚀 快速开始

### 方式 1: ROS2（推荐用于真实机器人）

```bash
# 1. 创建 Python 3.10 环境包括后面的dora环境也是建议使用py310版本，ros2版本的要求
# 其实所有的环境先装着，然后按照这个文件
conda create -n ros2_env python=3.10 -y
conda activate ros2_env

# 2. 一键启动
cd ROS_Module/ros2
./start_ros2_mcp.sh
```

**测试指令：**
```
先左转90度，再往前走1米
前进50厘米然后向右转45度
抓取杯子
```

### 方式 2: Dora（推荐用于仿真测试）

```bash
# 1. 安装依赖
pip install dora-rs pyarrow pyyaml python-dotenv

# 2. 启动 Dora
cd Dora_Module
dora up

# 3. 运行
dora start dora-interactive-mcp.yaml --attach
```

## 📊 功能对比

| 特性 | ROS2 | Dora |
|------|------|------|
| 真实机器人 | ✅ | ❌ |
| 仿真环境 | ✅ | ✅ |
| 双层LLM | ✅ | ✅ |
| 可视化 | rqt_graph | Rerun |
| 难度 | 中等 | 简单 |

## 📚 模块文档

- **[MCP_Server](MCP_Server/README.md)** - MCP 服务器和技能库
- **[ROS_Module](ROS_Module/README.md)** - ROS2 通信和控制
- **[Dora_Module](Dora_Module/README.md)** - Dora 仿真和配置

## 🔧 环境要求

- Python 3.10（ROS2 Humble 要求）
- ROS2 Humble（可选，用于真实机器人）
- Dora（可选，用于仿真）
- 阿里云通义千问 API Key

## 💡 核心概念

```
用户输入: "先左转90度，再往前走1米"
  ↓
🧠 上层 LLM (任务规划)
  ↓
分解: [左转90度] → [前进1米]
  ↓
⚙️  下层 LLM (执行)
  ↓
调用工具: turn_left(90) → move_forward(1.0m)
  ↓
📤 通信层 (ROS2/Dora)
  ↓
🤖 机器人执行
```

## 📝 配置

在项目根目录创建 `.env` 文件：

```bash
Test_API_KEY=sk-your-api-key-here
```

获取 API Key: https://dashscope.aliyun.com

## 🆚 与传统方案对比

| 指令 | 单层 LLM | 双层 LLM (本系统) |
|------|---------|-------------------|
| 先左转90度，再往前走1米 | ❌ 只执行左转 | ✅ 左转 → 前进 |
| 前进50cm然后向右转 | ❌ 只执行前进 | ✅ 前进 → 右转 |

## 📞 获取帮助

- ROS2 问题: 查看 [ROS_Module/README.md](ROS_Module/README.md)
- Dora 问题: 查看 [Dora_Module/README.md](Dora_Module/README.md)
- MCP 配置: 查看 [MCP_Server/README.md](MCP_Server/README.md)

## 📄 许可证

GPLv3+

---

**开始使用:** 选择 ROS2 或 Dora，查看对应模块的 README.md
