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
    DEFAULT_ENVTEST_ALIGNMENT = {
        "platform_1": None,
        "platform_2": None,
        "box": None,
    }
    DEFAULT_OUTPUT = {
        "ground": "unknown",
        "left_side": "unknown",
        "right_side": "unknown",
        "front_area": "unknown",
        "obstacles": [],
        "suspected_height_diff": False,
        "uncertainties": [],
        "envtest_alignment": DEFAULT_ENVTEST_ALIGNMENT,
    }

    def __init__(
        self,
        api_key: str | None = None,
        base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
        model: str = "qwen3-vl-flash-2026-01-22",
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
    def _normalize_number(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _extract_height_meters(text: str) -> float | None:
        match = re.search(r"(\d+(?:\.\d+)?)\s*米", text)
        if not match:
            return None
        return float(match.group(1))

    @staticmethod
    def _is_clear_path(text: str) -> bool:
        return any(keyword in text for keyword in ("无障碍", "可通行", "畅通", "开阔"))

    @staticmethod
    def _infer_side_from_y(y_value: Any) -> str:
        try:
            return "left" if float(y_value) >= 0 else "right"
        except (TypeError, ValueError):
            return "unknown"

    @staticmethod
    def _normalize_alignment_name(name: Any) -> str:
        normalized = str(name or "").strip().lower()
        allowed = {
            "left_low_obstacle",
            "left_high_obstacle",
            "right_low_obstacle",
            "right_high_obstacle",
            "support_box",
            "unknown_platform",
            "unknown_box",
        }
        return normalized if normalized in allowed else "unknown"

    @classmethod
    def _normalize_alignment_slot(cls, payload: Any, slot_name: str) -> dict[str, Any] | None:
        if payload is None:
            return None
        if not isinstance(payload, dict):
            return None

        normalized_type = str(payload.get("type", "")).strip().lower() or ("box" if slot_name == "box" else "platform")
        normalized_side = str(payload.get("side", "")).strip().lower()
        if normalized_side not in {"left", "right", "center", "unknown"}:
            normalized_side = "center" if slot_name == "box" else ("left" if slot_name == "platform_1" else "right")

        return {
            "name": cls._normalize_alignment_name(payload.get("name")),
            "type": normalized_type if normalized_type in {"platform", "box", "unknown"} else "unknown",
            "side": normalized_side,
            "height_m": round(cls._normalize_number(payload.get("height_m"), 0.0), 3),
            "summary": cls._normalize_text(payload.get("summary")),
        }

    @classmethod
    def _normalize_envtest_alignment(cls, payload: Any) -> dict[str, Any]:
        normalized = dict(cls.DEFAULT_ENVTEST_ALIGNMENT)
        if not isinstance(payload, dict):
            return normalized

        normalized["platform_1"] = cls._normalize_alignment_slot(payload.get("platform_1"), "platform_1")
        normalized["platform_2"] = cls._normalize_alignment_slot(payload.get("platform_2"), "platform_2")
        normalized["box"] = cls._normalize_alignment_slot(payload.get("box"), "box")
        return normalized

    @staticmethod
    def _format_height_label(height: float) -> str:
        return f"{height:.1f}".rstrip("0").rstrip(".")

    @classmethod
    def build_scene_facts(cls, payload: dict[str, Any]) -> dict[str, Any]:
        left_text = cls._normalize_text(payload.get("left_side"))
        right_text = cls._normalize_text(payload.get("right_side"))
        front_text = cls._normalize_text(payload.get("front_area"))
        alignment = cls._normalize_envtest_alignment(payload.get("envtest_alignment"))

        terrain_features = []
        route_options = []
        interactive_objects = []

        left_alignment = alignment.get("platform_1")
        right_alignment = alignment.get("platform_2")
        box_alignment = alignment.get("box")

        left_height = (
            cls._normalize_number(left_alignment.get("height_m"), 0.0) if isinstance(left_alignment, dict) and left_alignment.get("height_m") is not None
            else cls._extract_height_meters(left_text)
        )
        right_height = (
            cls._normalize_number(right_alignment.get("height_m"), 0.0) if isinstance(right_alignment, dict) and right_alignment.get("height_m") is not None
            else cls._extract_height_meters(right_text)
        )
        left_height = left_height if left_height and left_height > 0 else None
        right_height = right_height if right_height and right_height > 0 else None

        if left_text != "unknown" or isinstance(left_alignment, dict):
            left_type = "platform" if left_height is not None or isinstance(left_alignment, dict) else "path"
            left_traversable = left_height is None or left_height <= cls.MAX_CLIMB_HEIGHT_M
            terrain_features.append(
                {
                    "side": "left",
                    "type": left_type,
                    "height_m": left_height or 0.0,
                    "traversable": left_traversable,
                    "description": left_alignment.get("summary") if isinstance(left_alignment, dict) and left_alignment.get("summary") != "unknown" else left_text,
                    "object_id": left_alignment.get("name") if isinstance(left_alignment, dict) else None,
                }
            )
            route_options.append(
                {
                    "direction": "left",
                    "status": "clear" if cls._is_clear_path(left_text) or left_traversable else "blocked",
                    "reason": left_alignment.get("summary") if isinstance(left_alignment, dict) and left_alignment.get("summary") != "unknown" else left_text,
                }
            )

        if right_text != "unknown" or isinstance(right_alignment, dict):
            right_type = "platform" if right_height is not None or isinstance(right_alignment, dict) else "path"
            right_traversable = right_height is None or right_height <= cls.MAX_CLIMB_HEIGHT_M
            terrain_features.append(
                {
                    "side": "right",
                    "type": right_type,
                    "height_m": right_height or 0.0,
                    "traversable": right_traversable,
                    "description": right_alignment.get("summary") if isinstance(right_alignment, dict) and right_alignment.get("summary") != "unknown" else right_text,
                    "object_id": right_alignment.get("name") if isinstance(right_alignment, dict) else None,
                }
            )
            route_options.append(
                {
                    "direction": "right",
                    "status": "clear" if cls._is_clear_path(right_text) or right_traversable else "blocked",
                    "reason": right_alignment.get("summary") if isinstance(right_alignment, dict) and right_alignment.get("summary") != "unknown" else right_text,
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
        if isinstance(box_alignment, dict):
            interactive_objects.append(
                {
                    "name": box_alignment.get("name", "support_box"),
                    "side": box_alignment.get("side", "center"),
                    "height_m": cls._normalize_number(box_alignment.get("height_m"), 0.0),
                    "movable": True,
                    "usable_as_step": cls._normalize_number(box_alignment.get("height_m"), 0.0) <= cls.MAX_CLIMB_HEIGHT_M,
                    "description": box_alignment.get("summary", "箱子"),
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
            "envtest_alignment": alignment,
        }

    @classmethod
    def build_scene_facts_from_object_facts(cls, object_facts: dict[str, Any]) -> dict[str, Any]:
        constraints = dict(object_facts.get("constraints") or {})
        climb_limit = float(constraints.get("max_climb_height_m", cls.MAX_CLIMB_HEIGHT_M))
        constraints["max_climb_height_m"] = climb_limit

        terrain_features = []
        interactive_objects = []
        route_options_by_side: dict[str, dict[str, Any]] = {}
        platform_summaries = []
        object_summaries = []

        for obj in object_facts.get("objects") or []:
            obj_id = cls._normalize_text(obj.get("id"))
            obj_type = cls._normalize_text(obj.get("type")).lower()
            center = obj.get("center") or [0.0, 0.0, 0.0]
            size = obj.get("size") or [0.0, 0.0, 0.0]
            side = cls._infer_side_from_y(center[1] if len(center) > 1 else 0.0)
            height = round(float(size[2]), 3)
            side_label = "左侧" if side == "left" else "右侧" if side == "right" else "当前侧"

            if obj_type == "platform":
                terrain_features.append(
                    {
                        "side": side,
                        "type": "platform",
                        "height_m": height,
                        "traversable": height <= climb_limit,
                        "description": f"{side_label}平台 {obj_id}",
                        "object_id": obj_id,
                    }
                )
                route_options_by_side[side] = {
                    "direction": side,
                    "status": "clear" if height <= climb_limit else "blocked",
                    "reason": f"{side_label}{cls._format_height_label(height)}米高台 {obj_id}",
                }
                platform_summaries.append(f"{side_label}存在约{cls._format_height_label(height)}米高台")
            elif obj_type == "box":
                interactive_objects.append(
                    {
                        "name": obj_id,
                        "side": side,
                        "height_m": height,
                        "movable": bool(obj.get("movable")),
                        "usable_as_step": height <= climb_limit,
                        "description": f"{side_label}箱子 {obj_id}",
                        "center": center,
                        "size": size,
                    }
                )
                route_options_by_side.setdefault(
                    side,
                    {
                        "direction": side,
                        "status": "clear",
                        "reason": f"{side_label}有可利用箱子 {obj_id}",
                    },
                )
                object_summaries.append(f"{side_label}有可推动箱子 {obj_id}")

        return {
            "summary": "，".join([*platform_summaries, *object_summaries]) if platform_summaries or object_summaries else "已加载结构化物体信息",
            "terrain_features": terrain_features,
            "interactive_objects": interactive_objects,
            "route_options": list(route_options_by_side.values()),
            "constraints": constraints,
            "uncertainties": [],
        }

    @classmethod
    def merge_scene_facts(
        cls,
        vlm_scene_facts: dict[str, Any] | None,
        object_facts: dict[str, Any] | None,
    ) -> dict[str, Any]:
        if not object_facts:
            return vlm_scene_facts or cls.build_scene_facts(cls.DEFAULT_OUTPUT)

        object_scene_facts = cls.build_scene_facts_from_object_facts(object_facts)
        if not vlm_scene_facts:
            return object_scene_facts

        merged = dict(object_scene_facts)
        merged["uncertainties"] = [
            *cls._normalize_list(object_scene_facts.get("uncertainties")),
            *cls._normalize_list(vlm_scene_facts.get("uncertainties")),
        ]
        merged["visual_summary"] = cls._normalize_text(vlm_scene_facts.get("summary"))
        return merged

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
        normalized["envtest_alignment"] = cls._normalize_envtest_alignment(payload.get("envtest_alignment"))
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
