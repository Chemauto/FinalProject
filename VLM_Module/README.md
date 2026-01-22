# 🤖 FinalProject - 基于双层LLM的机器人控制系统

基于双层 LLM 架构的机器人控制系统，采用 MCP (Model Context Protocol) 模块化设计，使用 **ROS2 话题**进行通信。

## 📑 目录

- [项目概述](#-项目概述)
- [快速开始](#-快速开始)
- [Interactive_Module - 交互界面](#interactive_module---交互界面)
- [LLM_Module - 双层LLM核心](#llm_module---双层llm核心)
- [Robot_Module - MCP工具注册中心](#robot_module---mcp工具注册中心)
- [Sim_Module - 2D仿真环境](#sim_module---2d仿真环境)
- [Test_Module - 追击功能](#test_module---追击功能)
- [VLM_Module - 视觉语言模型](#vlm_module---视觉语言模型)
- [Yolo_Module - YOLO目标检测](#yolo_module---yolo目标检测)
- [ROS2通讯](#-ros2通讯)
- [依赖安装](#-依赖安装)

---

## 🎯 项目概述

### 核心特性

- **双层 LLM 架构**: 任务规划 + 执行控制分离
- **MCP 模块化设计**: 参考 RoboOS，工具注册标准化
- **ROS2 通讯**: 使用 ROS2 话题，标准化通信
- **自然语言控制**: 支持中文指令控制机器人
- **2D 仿真**: 基于 Pygame 的轻量级仿真环境
- **追击功能**: 自动追击敌人，PID控制精确定位
- **视觉检测**: VLM 颜色检测与动作映射
- **懒加载设计**: 自动初始化，零配置

### 系统架构

```
用户输入: "追击敌人" 或 "前进1米然后左转90度"
    ↓
┌─────────────────────────────────────────────┐
│ ① Interactive_Module (交互界面)             │
│    - 接收用户自然语言指令                     │
│    - 协调 LLM 和 Robot_Module                │
│    - 显示执行结果                             │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ② LLM_Module - 上层LLM (任务规划)           │
│    输入: 用户指令 + 可用技能列表              │
│    输出: 子任务序列                           │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ② LLM_Module - 下层LLM (执行控制)           │
│    输入: 单个子任务描述                       │
│    输出: 工具调用                             │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│ ③ Robot_Module (MCP 工具注册中心)           │
│    ├─ skill.py: FastMCP 服务器              │
│    ├─ module/base.py: 移动/旋转/停止技能    │
│    ├─ module/chase.py: 追击技能             │
│    └─ module/vision.py: 视觉检测技能        │
└──────────────┬──────────────────────────────┘
               ↓
      ROS2 Topic (/robot/command, /robot/enemies, /robot/state)
         ↓ 发布 JSON 消息
┌─────────────────────────────────────────────┐
│ ④ Sim_Module (2D 仿真器)                    │
│    ├─ 订阅 ROS2 话题                         │
│    ├─ 机器人运动可视化                       │
│    ├─ 敌人管理 (spawn/move/remove)           │
│    └─ 状态发布 (robot/enemy positions)       │
└─────────────────────────────────────────────┘
```

### 项目结构

```
FinalProject/
├── README.md              # 本文档
├── requirements.txt       # Python 依赖
├── .env                   # API 密钥配置
│
├── Interactive_Module/    # 交互界面
│   └── interactive.py     # CLI 交互主程序
│
├── LLM_Module/            # 双层LLM核心
│   ├── llm_core.py        # LLMAgent: 规划 + 执行
│   └── prompts/           # YAML 提示词模板
│       └── planning_prompt_2d.yaml
│
├── Robot_Module/          # MCP 工具注册中心
│   ├── skill.py           # FastMCP 服务器入口
│   └── module/            # 功能模块
│       ├── base.py        # 底盘控制 (move/turn/stop)
│       ├── chase.py       # 追击功能 (chase_enemy)
│       └── vision.py      # 视觉检测 (detect_color)
│
├── Sim_Module/            # 仿真模块
│   └── sim2d/
│       └── simulator.py   # 主仿真器
│
├── Test_Module/           # 测试模块
│   ├── chase_core.py      # 追击算法核心
│   ├── enemy_manager.py   # 敌人管理器
│   └── chase_simulator.py # 追击仿真器 (简化版)
│
├── VLM_Module/            # 视觉语言模型
│   ├── vlm_core.py        # 本地 VLM (Ollama)
│   ├── vlm_core_remote.py # 远程 VLM (API)
│   └── prompts/           # VLM 提示词
│
├── Yolo_Module/           # YOLO目标检测
│   ├── target_detector.py # 目标检测器
│   ├── screen_capture.py  # 屏幕捕获
│   └── coordinate_mapper.py # 坐标映射
│
└── ros_topic_comm.py      # ROS2 通讯模块
```

---

## 🚀 快速开始

### 环境要求

- Python 3.10+
- Linux (推荐 Ubuntu 22.04)
- ROS2 Humble (可选，用于调试)

### 安装

```bash
# 1. 进入项目目录
cd /home/xcj/work/testfinal/FinalProject

# 2. 安装 Python 依赖
pip install -r requirements.txt

# 3. 配置 API Key
export Test_API_KEY=your_api_key_here

# 或创建 .env 文件
echo "Test_API_KEY=your_api_key_here" > .env
```

### 运行

```bash
# 方式1: 一键启动（推荐）
./start_robot_system.sh

# 方式2: 手动启动
# 终端1: 启动仿真器
python3 Sim_Module/sim2d/simulator.py

# 终端2: 启动交互程序
python3 Interactive_Module/interactive.py
```

### 交互示例

```bash
💬 请输入指令: 前进1米然后左转90度

[上层LLM] 任务规划:
  步骤1: 前进1米
  步骤2: 左转90度

[下层LLM] 执行控制:
  调用工具: move_forward(distance=1.0) ✅
  调用工具: turn(angle=90.0) ✅

📊 [完成] 2/2 个任务成功
```

---


====================================
各模块详细文档
====================================

## Interactive_Module - 交互界面
# Interactive_Module - 交互界面模块

用户交互界面的核心模块，提供 CLI 命令行交互功能，协调 LLM 和 Robot_Module。

## 📁 模块结构

```
Interactive_Module/
├── README.md              # 本文档
└── interactive.py         # CLI 交互主程序
```

## 🎯 功能特性

- **自然语言交互**: 支持中文指令输入

====================================
VLM_Module 模块详细文档
====================================

# VLM_Module - 视觉语言模型模块

提供视觉感知能力，支持图像理解和颜色检测。

## 📁 模块结构

```
VLM_Module/
├── README.md                  # 本文档
├── __init__.py
├── vlm_core.py                # 本地 VLM (Ollama)
├── vlm_core_remote.py         # 远程 VLM (API)
├── assets/                    # 资源文件
└── prompts/                   # VLM 提示词
    └── perceive_environment.yaml
```

## 🎯 功能特性

- **双模式支持**: 本地 Ollama + 远程 API
- **颜色检测**: 识别图像中的主要颜色
- **动作映射**: 根据检测结果映射到机器人动作
- **灵活配置**: 支持多种 VLM 模型
- **错误处理**: 完善的异常处理和重试机制

## 🔧 核心组件

### vlm_core.py - 本地 VLM (Ollama)

**VLMCore 类：**

```python
class VLMCore:
    """本地视觉语言模型核心（Ollama）"""

    def __init__(self, model_name="llava"):
        """
        初始化本地 VLM

        Args:
            model_name: Ollama 模型名称
        """
        self.model_name = model_name

    def perceive_image(self, image_path: str) -> str:
        """
        感知图像内容

        Args:
            image_path: 图像文件路径

        Returns:
            图像描述文本
        """

    def detect_color(self, image_path: str) -> str:
        """
        检测图像中的主要颜色

        Args:
            image_path: 图像文件路径

        Returns:
            颜色名称（如 "红色"、"蓝色"）
        """
```

**使用示例：**

```python
from VLM_Module.vlm_core import VLMCore

# 初始化本地 VLM
vlm = VLMCore(model_name="llava")

# 感知图像
description = vlm.perceive_image("/path/to/image.jpg")
print(description)
# "一只红色的球在绿色的草地上"

# 检测颜色
color = vlm.detect_color("/path/to/image.jpg")
print(color)
# "红色"
```

**支持的 Ollama 模型：**

- `llava` - LLaVA (推荐)
- `llava-llama3` - LLaVA-Llama3
- `bakllava` - BakLLaVA
- 其他兼容 Ollama 的 VLM 模型

### vlm_core_remote.py - 远程 VLM (API)

**VLMCoreRemote 类：**

```python
class VLMCoreRemote:
    """远程视觉语言模型核心（API）"""

    def __init__(self, api_key: str, base_url: str, model: str = "qwen-vl-plus"):
        """
        初始化远程 VLM

        Args:
            api_key: API 密钥
            base_url: API 基础 URL
            model: 模型名称
        """

    def perceive_image(self, image_path: str) -> str:
        """感知图像内容"""

    def detect_color(self, image_path: str) -> str:
        """检测图像中的主要颜色"""
```

**使用示例：**

```python
from VLM_Module.vlm_core_remote import VLMCoreRemote

# 初始化远程 VLM
vlm = VLMCoreRemote(
    api_key="your_api_key",
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    model="qwen-vl-plus"
)

# 检测颜色
color = vlm.detect_color("/path/to/image.jpg")
print(color)
# "红色"
```

**支持的 API 模型：**

- `qwen-vl-plus` - 通义千问 VL Plus（推荐）
- `qwen-vl-max` - 通义千问 VL Max
- `gpt-4-vision` - GPT-4 Vision
- 其他兼容 OpenAI API 的 VLM

### prompts/perceive_environment.yaml - 提示词模板

**提示词内容：**

```yaml
system_prompt: |
  你是一个机器人视觉感知助手。你的任务是分析图像并提供准确的信息。

  重点关注：
  - 图像中的主要颜色
  - 物体的位置和方向
  - 可能的障碍物

prompt: |
  请分析这张图像，回答以下问题：
  1. 图像中的主要颜色是什么？（选择：红色、蓝色、绿色、黄色、黑色、白色）
  2. 这个颜色在图像的什么位置？（左、右、上、下、中）

  请简洁回答，只说颜色名称。
```

## 🎨 颜色检测与动作映射

### 颜色动作映射表

```python
COLOR_ACTION_MAP = {
    "红色": {
        "action": "turn",
        "parameters": {"angle": -90.0}  # 左转 90 度
    },
    "蓝色": {
        "action": "turn",
        "parameters": {"angle": 90.0}   # 右转 90 度
    },
    "绿色": {
        "action": "move_forward",
        "parameters": {"distance": 0.5}  # 前进 0.5 米
    },
    "黄色": {
        "action": "stop",
        "parameters": {}
    },
}
```

### 完整流程

```python
# 1. 检测颜色
color = vlm.detect_color("/path/to/image.jpg")

# 2. 映射到动作
if color in COLOR_ACTION_MAP:
    action_config = COLOR_ACTION_MAP[color]

    # 3. 发送命令
    from ros_topic_comm import get_shared_queue
    queue = get_shared_queue()
    queue.put(action_config)

    print(f"检测到 {color}，执行动作: {action_config['action']}")
```

## 🚀 快速开始

### 方式 1: 使用本地 VLM (Ollama)

```bash
# 1. 安装 Ollama
curl -fsSL https://ollama.com/install.sh | sh

# 2. 下载模型
ollama pull llava

# 3. 运行
python3 -c "
from VLM_Module.vlm_core import VLMCore
vlm = VLMCore()
print(vlm.detect_color('/path/to/image.jpg'))
"
```

### 方式 2: 使用远程 VLM (API)

```python
from VLM_Module.vlm_core_remote import VLMCoreRemote
import os

# 初始化
vlm = VLMCoreRemote(
    api_key=os.getenv("Test_API_KEY"),
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    model="qwen-vl-plus"
)

# 检测颜色
color = vlm.detect_color("/path/to/image.jpg")
print(f"检测到颜色: {color}")
```

### 方式 3: 在交互程序中使用

```bash
# 启动交互程序
python3 Interactive_Module/interactive.py

# 输入命令
根据 /home/robot/image.png 检测颜色并执行动作
```

## 🔧 配置说明

### Ollama 配置

```bash
# 查看已安装的模型
ollama list

# 下载新模型
ollama pull llava-llama3

# 运行模型
ollama run llava
```

### API 配置

环境变量（.env 文件）：

```bash
# API 密钥
Test_API_KEY=your_api_key_here

# VLM 模型
VLM_MODEL=qwen-vl-plus
VLM_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
```

### 模型选择指南

**本地 VLM (Ollama):**
- ✅ 免费、隐私、低延迟
- ❌ 需要本地资源、模型能力有限
- 适合: 开发测试、离线环境

**远程 VLM (API):**
- ✅ 模型能力强、无需本地资源
- ❌ 需要网络、有 API 成本
- 适合: 生产环境、高精度需求

## 💡 使用场景

### 场景 1: 颜色导航

```python
# 机器人根据颜色标志导航
color = vlm.detect_color("/robot/camera/snapshot.jpg")

if color == "红色":
    # 左转
    queue.put({"action": "turn", "parameters": {"angle": -90}})
elif color == "蓝色":
    # 右转
    queue.put({"action": "turn", "parameters": {"angle": 90}})
```

### 场景 2: 物体识别

```python
# 识别图像中的物体
description = vlm.perceive_image("/robot/camera/snapshot.jpg")

if "球" in description:
    print("发现球，靠近物体")
    queue.put({"action": "move_forward", "parameters": {"distance": 0.5}})
```

### 场景 3: 障碍物检测

```python
# 检测前方障碍物
description = vlm.perceive_image("/robot/camera/front.jpg")

if "障碍物" in description or "阻挡" in description:
    print("检测到障碍物，停止")
    queue.put({"action": "stop", "parameters": {}})
```

## 🐛 调试技巧

### 1. 测试本地 VLM

```bash
# 使用 Ollama CLI
ollama run llava "描述这张图片: /path/to/image.jpg"
```

### 2. 测试 API 调用

```python
from VLM_Module.vlm_core_remote import VLMCoreRemote

vlm = VLMCoreRemote(api_key="your_key", ...)
result = vlm._call_api([
    {"type": "image_url", "image_url": "file:///path/to/image.jpg"}
])
print(result)
```

### 3. 查看提示词

```python
import yaml
with open("VLM_Module/prompts/perceive_environment.yaml") as f:
    prompt = yaml.safe_load(f)
    print(prompt['prompt'])
```

### 4. 调整颜色检测

编辑 `prompts/perceive_environment.yaml`：

```yaml
prompt: |
  请分析这张图像，只回答主要颜色：
  - 红色、蓝色、绿色、黄色、黑色、白色

  不要解释，直接说颜色名称。
```

## 🔗 相关模块

- `Robot_Module/module/vision.py` - 视觉检测工具
- `ros_topic_comm.py` - ROS2 通讯模块

## 📝 依赖

```
**本地 VLM (Ollama):**
ollama>=0.1.0  # Ollama 客户端

**远程 VLM (API):**
openai>=1.0.0  # OpenAI API（兼容其他 API）
pillow>=10.0.0 # 图像处理
```

## 🎯 性能优化

- **本地 VLM**: 使用 GPU 加速（Ollama 支持 CUDA）
- **远程 VLM**: 启用响应缓存
- **图像预处理**: 压缩图像大小（建议 < 1MB）
- **批量处理**: 合并多个图像请求

## 📊 模型对比

| 模型 | 类型 | 优势 | 劣势 |
|------|------|------|------|
| LLaVA | 本地 | 免费、隐私 | 能力一般 |
| Qwen-VL-Plus | 远程 | 强大、准确 | 需要网络 |
| GPT-4V | 远程 | 最强 | 成本高 |

---

**视觉感知，智能决策！** 👁️
