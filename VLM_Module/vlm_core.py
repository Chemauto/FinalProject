from __future__ import annotations
"""VLM 核心模块：读取图片并返回中文描述。"""

import json
from pathlib import Path
from typing import Any

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
    DEFAULT_OUTPUT = {
        "ground": "unknown",
        "left_side": "unknown",
        "right_side": "unknown",
        "front_area": "unknown",
        "obstacles": [],
        "suspected_height_diff": False,
        "uncertainties": [],
    }

    def __init__(
        self,
        api_key: str | None = None,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        model: str = "qwen3-vl-plus",
        temperature: float = 0.1,
        Video_Port: int | str = DEFAULT_VIDEO_PORT,
        default_image: str | None = DEFAULT_IMAGE_PATH,
    ):
        """初始化远程客户端与默认路径。"""
        self.root = Path(__file__).resolve().parent
        self.prompt_file = self.root / "prompts" / "VlmPrompt.yaml"
        self.model = model
        self.temperature = temperature
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

    @staticmethod
    def _strip_code_fence(text: str) -> str:
        stripped = text.strip()
        if not stripped.startswith("```"):
            return stripped

        lines = stripped.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        return "\n".join(lines).strip()

    @staticmethod
    def _normalize_text(value: Any) -> str:
        if value is None:
            return "unknown"
        text = str(value).strip()
        return text if text else "unknown"

    @classmethod
    def _normalize_list(cls, value: Any) -> list[str]:
        if value is None:
            return []
        if isinstance(value, list):
            return [cls._normalize_text(item) for item in value if cls._normalize_text(item) != "unknown"]
        text = cls._normalize_text(value)
        return [] if text == "unknown" else [text]

    @staticmethod
    def _normalize_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "yes", "y", "是"}:
                return True
            if normalized in {"false", "0", "no", "n", "否"}:
                return False
        return False

    @classmethod
    def _normalize_payload(cls, payload: dict[str, Any]) -> dict[str, Any]:
        normalized = dict(cls.DEFAULT_OUTPUT)
        normalized["ground"] = cls._normalize_text(payload.get("ground"))
        normalized["left_side"] = cls._normalize_text(payload.get("left_side"))
        normalized["right_side"] = cls._normalize_text(payload.get("right_side"))
        normalized["front_area"] = cls._normalize_text(payload.get("front_area"))
        normalized["obstacles"] = cls._normalize_list(payload.get("obstacles"))
        normalized["suspected_height_diff"] = cls._normalize_bool(payload.get("suspected_height_diff"))
        normalized["uncertainties"] = cls._normalize_list(payload.get("uncertainties"))
        return normalized

    @classmethod
    def _parse_structured_output(cls, content: Any) -> dict[str, Any]:
        text = response_to_text(content)
        cleaned = cls._strip_code_fence(text)
        payload = json.loads(cleaned)
        if not isinstance(payload, dict):
            raise ValueError("VLM 输出不是 JSON 对象")
        return cls._normalize_payload(payload)

    def describe_structured(self, image_path: str | None = None) -> dict[str, Any]:
        """读取图片并调用远程 VLM，返回结构化环境事实。"""
        image_file = self.image_source.get_image(image_path)
        prompts = self._load_prompts()
        response = self.client.chat.completions.create(
            model=self.model,
            temperature=self.temperature,
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
        return self._parse_structured_output(response.choices[0].message.content)

    def describe(self, image_path: str | None = None) -> str:
        """读取图片并调用远程 VLM，返回 JSON 字符串。"""
        return json.dumps(self.describe_structured(image_path), ensure_ascii=False)




if __name__ == "__main__":
    print(VLMCore().describe())
