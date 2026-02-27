#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Core - 视觉语言模型核心模块

提供两种 VLM 实现方式：
1. 本地 Ollama（默认）：使用 ollama.run qwen3-vl:4b
2. 远程 API：使用 OpenAI 兼容 API（如 qwen-vl-plus）

用途：High-Level LLM 的环境理解功能
"""

import os
import sys
from typing import Optional


class VLMCore:
    """
    视觉语言模型核心类

    职责：
    1. 分析环境图像并生成描述性文本
    2. 支持本地 Ollama 和远程 API 两种模式
    3. 为 High-Level LLM 提供环境理解能力
    """

    def __init__(self,
                 vlm_prompt_path: Optional[str] = None,
                 default_image: str = "/home/xcj/work/FinalProject/VLM_Module/assets/red.png",
                 use_ollama: bool = True,
                 ollama_model: str = "qwen3-vl:4b",
                 ollama_host: str = "http://localhost:11434",
                 api_key: Optional[str] = None,
                 api_model: str = "qwen-vl-plus"):
        """
        初始化 VLM 核心

        Args:
            vlm_prompt_path: VLM 提示词文件路径（可选）
            default_image: 默认图片路径
            use_ollama: 是否使用本地 Ollama（默认 True）
            ollama_model: Ollama 模型名称（默认 qwen3-vl:4b）
            ollama_host: Ollama 服务地址（默认 localhost:11434）
            api_key: 远程 API 密钥（可选）
            api_model: 远程 API 模型名称（默认 qwen-vl-plus）
        """
        self.vlm_prompt_path = vlm_prompt_path
        self.default_image = default_image
        self.use_ollama = use_ollama
        self.ollama_model = ollama_model
        self.ollama_host = ollama_host
        self.api_key = api_key
        self.api_model = api_model

        # 加载 VLM 提示词
        self.vlm_prompt_template = self._load_vlm_prompt()

        # 懒加载客户端
        self._ollama_client = None
        self._api_client = None

        print(f"[VLM Core] 初始化完成", file=sys.stderr)
        print(f"  - 模式: {'本地 Ollama' if use_ollama else '远程 API'}", file=sys.stderr)
        print(f"  - 模型: {ollama_model if use_ollama else api_model}", file=sys.stderr)

    def _load_vlm_prompt(self) -> str:
        """从 YAML 文件加载 VLM 提示词"""
        if not self.vlm_prompt_path or not os.path.exists(self.vlm_prompt_path):
            return self._get_default_prompt()

        try:
            import yaml
            with open(self.vlm_prompt_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                return data.get("prompt", "")
        except Exception as e:
            print(f"⚠️  [VLM Core] 加载提示词失败: {e}，使用默认提示词", file=sys.stderr)
            return self._get_default_prompt()

    def _get_default_prompt(self) -> str:
        """获取默认的 VLM 环境理解提示词"""
        return """你是机器人的视觉感知助手。请分析这张环境图像，描述机器人看到的情况。

注意事项：
- 从机器人的视角描述环境
- 识别所有可见的物体和它们的相对位置
- 如果检测到颜色，明确说明颜色名称（如"红色方块"）
- 如果有明确的目标，说明其位置和距离
- 评估通道是否畅通
- 提供可行的移动建议

请用简洁的中文描述环境观察结果。"""

    def _get_ollama_client(self):
        """获取 Ollama 客户端（懒加载）"""
        if self._ollama_client is None:
            try:
                from ollama import Client
                self._ollama_client = Client(host=self.ollama_host)
                print(f"✅ [VLM Core] Ollama 客户端初始化成功: {self.ollama_host}", file=sys.stderr)
            except ImportError:
                print("⚠️  [VLM Core] ollama 包未安装，请运行: pip install ollama", file=sys.stderr)
                self._ollama_client = False
            except Exception as e:
                print(f"⚠️  [VLM Core] Ollama 客户端初始化失败: {e}", file=sys.stderr)
                self._ollama_client = False
        return self._ollama_client

    def _get_api_client(self):
        """获取远程 API 客户端（懒加载）"""
        if self._api_client is None:
            try:
                from openai import OpenAI
                if not self.api_key:
                    self.api_key = os.getenv('Test_API_KEY')
                if not self.api_key:
                    raise ValueError("未提供 API 密钥")
                self._api_client = OpenAI(
                    api_key=self.api_key,
                    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
                )
                print(f"✅ [VLM Core] API 客户端初始化成功", file=sys.stderr)
            except Exception as e:
                print(f"⚠️  [VLM Core] API 客户端初始化失败: {e}", file=sys.stderr)
                self._api_client = False
        return self._api_client

    def analyze_environment(self, image_path: Optional[str] = None) -> Optional[str]:
        """
        分析环境图像

        Args:
            image_path: 图像文件路径（可选，默认使用 red.png）

        Returns:
            环境理解文本，失败时返回 None
        """
        # 使用默认图片或用户提供的图片
        image_path = image_path or self.default_image

        if not os.path.exists(image_path):
            print(f"⚠️  [VLM Core] 图像文件不存在: {image_path}", file=sys.stderr)
            return None

        print(f"🖼️  [VLM Core] 分析环境图像: {image_path}", file=sys.stderr)

        # 根据模式选择分析方法
        if self.use_ollama:
            return self._analyze_with_ollama(image_path)
        else:
            return self._analyze_with_api(image_path)

    def _analyze_with_ollama(self, image_path: str) -> Optional[str]:
        """使用本地 Ollama VLM 分析图像"""
        client = self._get_ollama_client()

        if not client or client is False:
            print("⚠️  [VLM Core] Ollama 客户端不可用", file=sys.stderr)
            return None

        try:
            response = client.chat(
                model=self.ollama_model,
                messages=[
                    {'role': 'user', 'content': self.vlm_prompt_template, 'images': [image_path]},
                    {'role': 'system', 'content': '请始终使用简体中文进行回复。'}
                ]
            )

            result = response['message']['content'].strip()
            print(f"✅ [VLM Core] 环境理解完成 (Ollama: {self.ollama_model})", file=sys.stderr)
            return result

        except Exception as e:
            print(f"⚠️  [VLM Core] Ollama 图像分析失败: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            return None

    def _analyze_with_api(self, image_path: str) -> Optional[str]:
        """使用远程 OpenAI 兼容 API 分析图像"""
        import base64

        client = self._get_api_client()

        if not client or client is False:
            print("⚠️  [VLM Core] API 客户端不可用", file=sys.stderr)
            return None

        try:
            # 读取图片并转换为 base64
            with open(image_path, 'rb') as img_file:
                image_data = base64.b64encode(img_file.read()).decode('utf-8')

            # 构造 VLM 消息
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self.vlm_prompt_template},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/png;base64,{image_data}"
                            }
                        }
                    ]
                }
            ]

            # 调用 VLM API
            completion = client.chat.completions.create(
                model=self.api_model,
                messages=messages,
                temperature=0.3
            )

            result = completion.choices[0].message.content.strip()
            print(f"✅ [VLM Core] 环境理解完成 (API: {self.api_model})", file=sys.stderr)
            return result

        except Exception as e:
            print(f"⚠️  [VLM Core] API 图像分析失败: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            return None


# 便捷函数
def create_vlm_core(
    vlm_prompt_path: Optional[str] = None,
    default_image: str = "/home/xcj/work/FinalProject/VLM_Module/assets/red.png",
    use_ollama: bool = True,
    ollama_model: str = "qwen3-vl:4b",
    ollama_host: str = "http://localhost:11434",
    api_key: Optional[str] = None,
    api_model: str = "qwen-vl-plus"
) -> VLMCore:
    """
    创建 VLM 核心实例

    Args:
        vlm_prompt_path: VLM 提示词文件路径
        default_image: 默认图片路径
        use_ollama: 是否使用本地 Ollama
        ollama_model: Ollama 模型名称
        ollama_host: Ollama 服务地址
        api_key: 远程 API 密钥
        api_model: 远程 API 模型名称

    Returns:
        VLMCore 实例
    """
    return VLMCore(
        vlm_prompt_path=vlm_prompt_path,
        default_image=default_image,
        use_ollama=use_ollama,
        ollama_model=ollama_model,
        ollama_host=ollama_host,
        api_key=api_key,
        api_model=api_model
    )

