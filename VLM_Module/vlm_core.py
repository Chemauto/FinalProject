import sys
from ollama import Client
import yaml
import json

class VLMCore:
    def __init__(self, model='qwen3-vl:4b', host='http://localhost:11434'):
        self.client = Client(host=host)
        self.model = model
        self.default_image = '/home/robot/work/FinalProject/VLM_Module/assets/red.png'  # 默认红色
        self.prompt_path = '/home/robot/work/FinalProject/VLM_Module/prompts/perceive_environment.yaml'

    def perceive(self, image_path=None):
        """识别颜色，返回格式化结果供LLM使用"""
        image_path = image_path or self.default_image

        with open(self.prompt_path, 'r', encoding='utf-8') as f:
            prompt = yaml.safe_load(f)['prompt']

        response = self.client.chat(
            model=self.model,
            messages=[
                {'role': 'user', 'content': prompt, 'images': [image_path]},
                {'role': 'system', 'content': '请始终使用简体中文进行回复。'}
            ]
        )

        content = response['message']['content']
        print(f"[VLM] 原始响应: {content}", file=sys.stderr)

        # 提取JSON（去除markdown代码块）
        if '```' in content:
            for block in content.split('```'):
                if '{' in block and '}' in block:
                    content = block.strip()
                    if content.startswith('json'):
                        content = content[4:].strip()
                    break

        try:
            result = json.loads(content)
            color = result.get('color', 'none').lower()
            print(f"[VLM] 识别颜色: {color}", file=sys.stderr)

            # 颜色到动作的映射
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
                print(f"[VLM] 未知颜色: {color}", file=sys.stderr)
                return None

            action = color_action_map[color]
            if action == 'none':
                print(f"[VLM] {color}色无动作", file=sys.stderr)
                return None

            return {'vlm_input': f'检测到{color}色方块', 'action': action}
        except Exception as e:
            print(f"[VLM] JSON解析失败: {e}", file=sys.stderr)
            return None
