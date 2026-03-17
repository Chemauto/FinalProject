from __future__ import annotations
"""VLM 核心模块：读取图片并返回中文描述。"""

from pathlib import Path

from openai import OpenAI
import yaml

try:
    from .image_source import ImageSource, Video_Port as DEFAULT_VIDEO_PORT, image_path as DEFAULT_IMAGE_PATH
    from .vlm_utils import load_api_key, response_to_text, to_data_url
except ImportError:
    from image_source import ImageSource, Video_Port as DEFAULT_VIDEO_PORT, image_path as DEFAULT_IMAGE_PATH
    from vlm_utils import load_api_key, response_to_text, to_data_url


class VLMCore:
    """远程视觉描述器，只负责提示词读取与模型调用。"""

    def __init__(
        self,
        api_key: str | None = None,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        model: str = "qwen3-vl-plus",
        Video_Port: int | str = DEFAULT_VIDEO_PORT,
        default_image: str | None = DEFAULT_IMAGE_PATH,
    ):
        """初始化远程客户端与默认路径。"""
        self.root = Path(__file__).resolve().parent
        self.prompt_file = self.root / "prompts" / "VlmPrompt.yaml"
        self.model = model
        self.image_source = ImageSource(Video_Port=Video_Port, default_image=default_image)
        self.client = OpenAI(api_key=api_key or load_api_key(self.root), base_url=base_url)
    
    def _load_prompts(self) -> dict[str, str]:
        """从 YAML 中读取 system 和 user 提示词。"""
        data = yaml.safe_load(self.prompt_file.read_text(encoding="utf-8")) or {}
        system_prompt = str(data.get("system_prompt", "")).strip()
        user_prompt = str(data.get("user_prompt", "")).strip()
        if not system_prompt or not user_prompt:
            raise ValueError(f"提示词为空: {self.prompt_file}")
        return {"system": system_prompt, "user": user_prompt}
    
    def describe(self, image_path: str | None = None) -> str:
        """读取图片并调用远程 VLM，返回中文描述文本。"""
        image_file = self.image_source.get_image(image_path)
        prompts = self._load_prompts()
        response = self.client.chat.completions.create(
            model=self.model,
            temperature=0.3,
            messages=[
                {
                    "role": "system",
                    "content": prompts["system"],
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": prompts["user"],
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": to_data_url(image_file)},
                        },
                    ],
                },
            ],
        )
        return response_to_text(response.choices[0].message.content)




if __name__ == "__main__":
    print(VLMCore().describe())
