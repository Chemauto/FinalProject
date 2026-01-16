#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM 远程API核心类 (Vision-Language Model Remote API Core)

支持使用远程VLM API进行视觉理解，兼容OpenAI API格式。
适用于：通义千问VL、OpenAI GPT-4o等支持视觉的API。
"""

import os
import sys
import json
import yaml
import base64
from pathlib import Path
from openai import OpenAI


class VLMRemote:
    """远程VLM客户端（基于OpenAI兼容API）"""

    def __init__(
        self,
        api_key: str = None,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        model: str = "qwen-vl-plus"
    ):
        """初始化远程VLM客户端

        Args:
            api_key: API密钥（默认从.env文件的Test_API_KEY读取）
            base_url: API端点URL
            model: 模型名称（qwen-vl-plus或qwen-vl-max）
        """
        # 从.env文件读取API密钥
        if api_key is None:
            env_path = Path(__file__).parent.parent / '.env'
            if env_path.exists():
                from dotenv import load_dotenv
                load_dotenv(env_path)
                api_key = os.getenv('Test_API_KEY')

        if not api_key:
            raise ValueError("API密钥未找到，请确保.env文件中有Test_API_KEY")

        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = model
        self.prompt_path = Path(__file__).parent / "prompts" / "perceive_environment.yaml"
        self.default_image = Path(__file__).parent / "assets" / "red.png"

        print(f"[VLM Remote] 初始化完成: model={model}", file=sys.stderr)

    def perceive(self, image_path: str = None) -> dict:
        """识别图片颜色并返回对应动作

        Args:
            image_path: 图片路径（可选，默认使用red.png）

        Returns:
            识别结果字典 {'color': 'red', 'action': 'move_forward'}
            无有效颜色时返回 None
        """
        image_path = image_path or self.default_image

        # 读取提示词
        with open(self.prompt_path, 'r', encoding='utf-8') as f:
            prompt = yaml.safe_load(f)['prompt']

        # 读取图片并转换为base64
        with open(image_path, 'rb') as img_file:
            image_data = base64.b64encode(img_file.read()).decode('utf-8')

        # 构造消息（OpenAI视觉API格式）
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/png;base64,{image_data}"
                        }
                    }
                ]
            },
            {
                "role": "system",
                "content": "请始终使用简体中文进行回复。只返回JSON格式，不要其他内容。"
            }
        ]

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.3
            )

            content = response.choices[0].message.content.strip()
            print(f"[VLM Remote] 原始响应: {content}", file=sys.stderr)

            # 提取JSON（去除markdown代码块）
            if '```' in content:
                for block in content.split('```'):
                    if '{' in block and '}' in block:
                        content = block.strip()
                        if content.startswith('json'):
                            content = content[4:].strip()
                        break

            result = json.loads(content)
            color = result.get('color', 'none').lower()
            print(f"[VLM Remote] 识别颜色: {color}", file=sys.stderr)

            # 颜色到动作映射
            color_action_map = {
                'red': 'move_forward',
                'orange': 'move_forward',
                'yellow': 'turn_left',
                'green': 'move_backward',
                'blue': 'turn_right',
                'purple': 'stop',
                'black': 'none'
            }

            if color not in color_action_map:
                print(f"[VLM Remote] 未知颜色: {color}", file=sys.stderr)
                return None

            action = color_action_map[color]
            if action == 'none':
                print(f"[VLM Remote] {color}色无动作", file=sys.stderr)
                return None

            return {'color': color, 'action': action}

        except Exception as e:
            print(f"[VLM Remote] API调用失败: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            return None


# 兼容性别名：VLMCore -> VLMRemote
VLMCore = VLMRemote


# 测试代码
if __name__ == "__main__":
    vlm = VLMRemote()

    # 测试红色图片
    print("\n=== 测试红色图片 ===")
    result = vlm.perceive('/home/robot/work/FinalProject/VLM_Module/assets/red.png')
    print(f"结果: {result}")

    # 测试绿色图片
    print("\n=== 测试绿色图片 ===")
    result = vlm.perceive('/home/robot/work/FinalProject/VLM_Module/assets/green.png')
    print(f"结果: {result}")
