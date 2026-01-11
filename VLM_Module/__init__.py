"""
VLM Module - 视觉语言模型模块

功能:
- 环境感知 (图像输入 → 环境描述)
- 障碍物检测 (图像输入 → 障碍物列表)
- 导航建议 (图像+目标 → 建议路径)

提示词位置:
- prompts/*.yaml (YAML格式的视觉提示词)
"""

from .vlm_core import VLMCore

__version__ = '1.0.0'
__all__ = ['VLMCore']
