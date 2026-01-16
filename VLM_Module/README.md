# VLM_Module - 视觉语言模型模块

## 概述

VLM_Module 提供了**视觉语言模型 (Vision-Language Model)** 集成能力，使机器人能够理解和分析视觉信息，支持环境感知、障碍物检测和导航决策。

### 核心功能

- **环境感知**: 分析图像并生成环境描述
- **障碍物检测**: 识别图像中的障碍物及其位置
- **导航建议**: 基于视觉输入提供导航决策
- **场景分析**: 理解场景内容、物体和空间关系

### 当前状态

✅ **框架完成**: VLMCore 类已完全实现
⚠️ **待集成**: 需要将其集成到主控制流程中
⚠️ **提示词优化**: 提示词模板需要根据具体应用优化

## 架构与数据流

```
摄像头/图像输入
    ↓
┌─────────────────────────────────────────┐
│ VLM_Module.vlm_core.py                  │
│                                         │
│ 1. 图像编码 (base64)                    │
│ 2. 提示词加载                           │
│ 3. VLM API 调用                         │
│ 4. 结果解析                             │
└──────────────┬──────────────────────────┘
               ↓
      视觉分析结果 (文本/结构化数据)
               ↓
      (可选) 传递给 LLM_Module 进行决策
```

## 文件结构

```
VLM_Module/
├── __init__.py           # 模块导出
├── vlm_core.py           # VLMCore 核心类实现
└── prompts/              # VLM 视觉提示词模板
    └── perceive_environment.yaml  # 环境感知提示词
```

## 核心类: VLMCore

### 初始化

```python
from VLM_Module import VLMCore

# 使用默认配置
vlm = VLMCore()

# 使用自定义配置文件
vlm = VLMCore(config_path="path/to/vlm_config.yaml")
```

### 配置文件格式

`config/vlm_config.yaml` (可选):

```yaml
provider: openai                    # openai, anthropic, or local
model: gpt-4-vision-preview        # 模型名称
api_key: ${OPENAI_API_KEY}         # API 密钥
max_tokens: 1000                    # 最大生成 token 数
```

## 核心方法

### 1. `analyze_image(image_path, task)` - 通用图像分析

使用 VLM 分析图像并返回结果。

```python
result = vlm.analyze_image(
    image_path="camera_image.jpg",
    task="describe"  # 任务类型
)

# 返回格式:
# {
#     'success': True,
#     'result': '图像分析结果文本...',
#     'provider': 'openai'
# }
```

**支持的任务类型**:
- `describe`: 描述图像内容
- `detect_objects`: 检测图像中的物体
- `perceive_environment`: 感知机器人环境
- `detect_obstacles`: 检测障碍物
- `suggest_navigation`: 导航建议
- 自定义任务名称

### 2. `perceive_environment(image_path)` - 环境感知

分析机器人周围环境并生成描述。

```python
perception = vlm.perceive_environment("camera_image.jpg")

# 返回环境感知结果
print(perception['result'])
# 输出示例: "这是一个室内走廊环境，前方有一条笔直的通道，
#           两侧有墙壁，地面平坦，没有明显障碍物..."
```

### 3. `detect_obstacles(image_path)` - 障碍物检测

检测环境中的障碍物。

```python
obstacles = vlm.detect_obstacles("camera_image.jpg")

# 返回检测到的障碍物列表
# [
#     {'type': 'chair', 'position': 'left', 'distance': '1.5m'},
#     {'type': 'box', 'position': 'center', 'distance': '3m'}
# ]
```

### 4. `suggest_navigation(image_path, goal)` - 导航建议

基于视觉输入和目标提供导航建议。

```python
suggestion = vlm.suggest_navigation(
    image_path="camera_image.jpg",
    goal="到达走廊尽头的门"
)

# 返回导航建议
print(suggestion['result'])
# 输出示例: "建议向前直线移动约5米到达门口，
#           注意避开左侧1.5米处的椅子..."
```

## 提示词系统

VLM_Module 使用 YAML 格式的提示词模板，支持变量替换：

```yaml
# prompts/perceive_environment.yaml
prompt: |
  你是一个机器人环境感知助手。
  请分析这张来自机器人摄像头的图像。

  图像路径：{image_path}

  请提供：
  1. 环境类型（室内/室外）
  2. 主要物体和障碍物
  3. 空间布局
  4. 可通行路径建议
```

## 使用示例

### 基础使用

```python
from VLM_Module import VLMCore

# 初始化
vlm = VLMCore()

# 分析图像
result = vlm.analyze_image("scene.jpg", task="describe")

if result['success']:
    print("分析结果:", result['result'])
else:
    print("分析失败:", result['error'])
```

### 与 LLM 集成

```python
from VLM_Module import VLMCore
from LLM_Module import LLMAgent

# 初始化
vlm = VLMCore()
llm = LLMAgent(api_key=os.getenv('Test_API_KEY'))

# 获取视觉感知
vision_result = vlm.perceive_environment('scene.jpg')

# 将视觉信息传递给 LLM 进行决策
user_input = f"根据摄像头画面: {vision_result['result']}，我应该怎么走？"
tasks = llm.plan_tasks(user_input, tools=[])
```

## 与系统集成

### 集成方式 1: 在 Interactive_Module 中集成

```python
from VLM_Module import VLMCore

# 在 interactive.py 中添加 VLM 支持
class InteractiveInterface:
    def __init__(self):
        self.vlm = VLMCore()  # 初始化 VLM

    def capture_and_analyze(self):
        """捕获并分析摄像头图像"""
        # 1. 捕获图像
        image_path = self.capture_image()

        # 2. 分析环境
        perception = self.vlm.perceive_environment(image_path)

        # 3. 将感知结果传递给 LLM
        return perception['result']
```

### 集成方式 2: 作为独立工具添加到 Robot_Module

```python
# Robot_Module/module/vision.py
async def analyze_environment(image_path: str) -> str:
    """分析环境图像

    Args:
        image_path: 图像文件路径

    Returns:
        环境描述字符串
    """
    from VLM_Module import VLMCore
    vlm = VLMCore()
    result = vlm.perceive_environment(image_path)
    return result['result']

def register_tools(mcp, tool_registry=None, tool_metadata=None):
    tools = [analyze_environment]
    # ... 注册逻辑
```

## 依赖

```yaml
# 核心依赖
yaml:           # 配置文件解析
base64:         # 图像编码
pathlib:        # 路径处理
typing:         # 类型注解

# 外部依赖 (按需安装)
openai:         # OpenAI API (GPT-4 Vision)
anthropic:      # Anthropic API (Claude)
```

安装命令:

```bash
# OpenAI GPT-4 Vision
pip install openai

# Anthropic Claude
pip install anthropic
```

## 错误处理

VLMCore 包含完善的错误处理机制：

```python
result = vlm.analyze_image('image.jpg', task='describe')

if not result['success']:
    if result['provider'] == 'openai':
        print(f"OpenAI API 错误: {result['error']}")
    elif result['provider'] == 'anthropic':
        print(f"Anthropic API 错误: {result['error']}")
```

## API 配置

### OpenAI (GPT-4 Vision)

```python
# 方法 1: 使用配置文件
vlm = VLMCore(config_path="config/vlm_config.yaml")

# 方法 2: 使用环境变量
import os
os.environ['OPENAI_API_KEY'] = 'sk-...'
vlm = VLMCore()

# 方法 3: 在配置中指定
vlm.config['api_key'] = 'sk-...'
```

### Anthropic (Claude)

```yaml
# config/vlm_config.yaml
provider: anthropic
model: claude-3-opus-20240229
api_key: ${ANTHROPIC_API_KEY}
max_tokens: 1000
```

## 扩展 VLM 模块

### 添加新的 VLM 提供商

```python
def _analyze_with_custom_provider(self, image_path: str, prompt: str):
    """自定义 VLM 提供商实现"""
    # 1. 编码图像
    base64_image = self._encode_image(image_path)

    # 2. 调用自定义 API
    # response = custom_api.call(...)

    # 3. 返回标准格式
    return {
        'success': True,
        'result': response,
        'provider': 'custom'
    }

# 在 analyze_image 中添加
elif provider == 'custom':
    return self._analyze_with_custom_provider(image_path, prompt)
```

### 添加新的分析任务

```python
def detect_humans(self, image_path: str) -> List[Dict]:
    """检测图像中的人"""
    result = self.analyze_image(image_path, task='detect_humans')
    # 解析结果并返回人员列表
    return self._parse_human_detection(result)
```

## 性能优化建议

1. **图像大小**: 发送前压缩图像以减少 API 调用成本
2. **缓存机制**: 对相同场景缓存分析结果
3. **异步调用**: 使用异步 API 提高响应速度
4. **批量处理**: 一次性分析多个图像

## 设计特点

1. **多提供商支持**: 支持 OpenAI 和 Anthropic
2. **模块化设计**: 易于添加新的 VLM 提供商
3. **灵活的任务系统**: 支持自定义分析任务
4. **完善的错误处理**: 统一的错误返回格式
5. **提示词模板**: YAML 格式的可配置提示词

## 相关文档

- [主项目 README](../README.md)
- [LLM_Module README](../LLM_Module/README.md)
- [Robot_Module README](../Robot_Module/README.md)

---

**视觉感知，智能决策！** 🚀
