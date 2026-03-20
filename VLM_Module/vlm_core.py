from __future__ import annotations
"""VLM 核心模块：读取图片并返回中文描述。"""

import json
import re
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
    MAX_CLIMB_HEIGHT_M = 0.3
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

    @staticmethod
    def _extract_height_meters(text: str) -> float | None:
        match = re.search(r"(\d+(?:\.\d+)?)\s*米", text)
        if not match:
            return None
        return float(match.group(1))

    @staticmethod
    def _is_clear_path(text: str) -> bool:
        return any(keyword in text for keyword in ("无障碍", "可通行", "畅通", "开阔"))

    @classmethod
    def build_scene_facts(cls, payload: dict[str, Any]) -> dict[str, Any]:
        left_text = cls._normalize_text(payload.get("left_side"))
        right_text = cls._normalize_text(payload.get("right_side"))
        front_text = cls._normalize_text(payload.get("front_area"))

        terrain_features = []
        route_options = []
        interactive_objects = []

        left_height = cls._extract_height_meters(left_text)
        right_height = cls._extract_height_meters(right_text)

        if left_text != "unknown":
            left_type = "platform" if left_height is not None else "path"
            left_traversable = left_height is None or left_height <= cls.MAX_CLIMB_HEIGHT_M
            terrain_features.append(
                {
                    "side": "left",
                    "type": left_type,
                    "height_m": left_height or 0.0,
                    "traversable": left_traversable,
                    "description": left_text,
                }
            )
            route_options.append(
                {
                    "direction": "left",
                    "status": "clear" if cls._is_clear_path(left_text) or left_traversable else "blocked",
                    "reason": left_text,
                }
            )

        if right_text != "unknown":
            right_type = "platform" if right_height is not None else "path"
            right_traversable = right_height is None or right_height <= cls.MAX_CLIMB_HEIGHT_M
            terrain_features.append(
                {
                    "side": "right",
                    "type": right_type,
                    "height_m": right_height or 0.0,
                    "traversable": right_traversable,
                    "description": right_text,
                }
            )
            route_options.append(
                {
                    "direction": "right",
                    "status": "clear" if cls._is_clear_path(right_text) or right_traversable else "blocked",
                    "reason": right_text,
                }
            )

        for obstacle in cls._normalize_list(payload.get("obstacles")):
            if "箱子" in obstacle or "box" in obstacle.lower():
                interactive_objects.append(
                    {
                        "name": "box",
                        "side": "left" if "左" in obstacle else "right" if "右" in obstacle else "unknown",
                        "height_m": cls._extract_height_meters(obstacle) or 0.0,
                        "movable": True,
                        "usable_as_step": True,
                        "description": obstacle,
                    }
                )

        summary_parts = []
        if left_height is not None:
            summary_parts.append(f"左侧存在约{left_height:.1f}米高台")
        elif left_text != "unknown":
            summary_parts.append(f"左侧{left_text}")
        if cls._is_clear_path(right_text):
            summary_parts.append("右侧可通行")
        elif right_height is not None:
            summary_parts.append(f"右侧存在约{right_height:.1f}米高差")
        elif right_text != "unknown":
            summary_parts.append(f"右侧{right_text}")
        if not summary_parts and front_text != "unknown":
            summary_parts.append(front_text)

        return {
            "summary": "，".join(summary_parts) if summary_parts else "环境信息不足",
            "terrain_features": terrain_features,
            "interactive_objects": interactive_objects,
            "route_options": route_options,
            "constraints": {"max_climb_height_m": cls.MAX_CLIMB_HEIGHT_M},
            "uncertainties": cls._normalize_list(payload.get("uncertainties")),
        }

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

    def describe_scene_facts(self, image_path: str | None = None) -> dict[str, Any]:
        """返回适合导航规划使用的场景事实。"""
        return self.build_scene_facts(self.describe_structured(image_path))

    def describe(self, image_path: str | None = None) -> str:
        """读取图片并调用远程 VLM，返回 JSON 字符串。"""
        return json.dumps(self.describe_structured(image_path), ensure_ascii=False)




if __name__ == "__main__":
    print(VLMCore().describe())
