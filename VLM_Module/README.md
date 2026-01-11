# VLM_Module - 视觉语言模型模块

## 功能

基于视觉语言模型的环境感知系统：
- **环境感知**: 图像 → 环境描述
- **障碍物检测**: 图像 → 障碍物列表
- **导航建议**: 图像 + 目标 → 建议路径

## 文件结构

```
VLM_Module/
├── __init__.py
├── vlm_core.py          # VLMCore 类实现
└── prompts/             # VLM 视觉提示词
    └── perceive_environment.yaml
```

## 核心类

### VLMCore

```python
from VLM_Module import VLMCore

vlm = VLMCore(config_path="path/to/config.yaml")

# 感知环境
environment = vlm.perceive_environment("camera_image.jpg")

# 检测障碍物
obstacles = vlm.detect_obstacles("camera_image.jpg")

# 分析图像
result = vlm.analyze_image("camera_image.jpg", "描述这个场景")
```

## 状态

⚠️ **待实现**: 当前为基础框架，等待视觉模型集成

## 依赖

- yaml
- base64
- openai (或其他 VLM API)
