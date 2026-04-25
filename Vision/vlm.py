import json
import os
import re
from pathlib import Path

import yaml
from openai import OpenAI

from Vision.image_source import ImageSource
from Vision.vlm_utils import load_api_key, response_to_text, to_data_url


class VisionCore:
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

    def __init__(self, api_key=None, base_url=None, model=None, temperature=0.1):
        self.root = Path(__file__).resolve().parent
        self.prompt_file = self.root / "prompts" / "VlmPrompt.yaml"
        self.model = model or os.getenv("VISION_MODEL", "qwen3-vl-flash-2026-01-22")
        self.temperature = temperature
        self.image_source = ImageSource()
        self.last_image_path = None
        self.client = OpenAI(
            api_key=api_key or load_api_key(self.root),
            base_url=base_url or os.getenv("MODEL_BASE_URL") or os.getenv("FINALPROJECT_BASE_URL"),
        )
    #初始化VLM客户端和图片来源

    def describe_structured(self, image_path=None):
        image_file = self.image_source.get_image(image_path)
        self.last_image_path = image_file
        prompts = self._load_prompts()
        response = self.client.chat.completions.create(
            model=self.model,
            temperature=self.temperature,
            messages=[
                {"role": "system", "content": prompts["system"]},
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompts["user"]},
                        {"type": "image_url", "image_url": {"url": to_data_url(image_file)}},
                    ],
                },
            ],
        )
        return self._parse_structured_output(response.choices[0].message.content)
    #调用VLM，把图片转成结构化视觉语义

    def _load_prompts(self):
        data = yaml.safe_load(self.prompt_file.read_text(encoding="utf-8")) or {}
        return {"system": str(data.get("system_prompt", "")).strip(), "user": str(data.get("user_prompt", "")).strip()}
    #读取Vision提示词

    @classmethod
    def build_scene_facts(cls, payload):
        left_text = cls._normalize_text(payload.get("left_side"))
        right_text = cls._normalize_text(payload.get("right_side"))
        front_text = cls._normalize_text(payload.get("front_area"))
        terrain_features = []
        route_options = []
        interactive_objects = []

        cls._add_side_fact("left", left_text, terrain_features, route_options)
        cls._add_side_fact("right", right_text, terrain_features, route_options)
        for obstacle in cls._normalize_list(payload.get("obstacles")):
            if "箱子" in obstacle or "box" in obstacle.lower():
                interactive_objects.append(cls._build_box_fact(obstacle))

        return {
            "summary": cls._build_summary(left_text, right_text, front_text),
            "terrain_features": terrain_features,
            "interactive_objects": interactive_objects,
            "route_options": route_options,
            "constraints": {"max_climb_height_m": cls.MAX_CLIMB_HEIGHT_M},
            "uncertainties": cls._normalize_list(payload.get("uncertainties")),
        }
    #把视觉语义整理成规划更容易使用的环境事实

    @classmethod
    def _add_side_fact(cls, side, text, terrain_features, route_options):
        if text == "unknown":
            return
        height = cls._extract_height_meters(text)
        wall = cls._is_wall_like(text)
        traversable = False if wall else (height is None or height <= cls.MAX_CLIMB_HEIGHT_M)
        terrain_features.append({
            "side": side,
            "type": "wall" if wall else "platform" if height is not None else "path",
            "height_m": height or 0.0,
            "traversable": traversable,
            "description": text,
        })
        route_options.append({
            "direction": side,
            "status": "clear" if cls._is_clear_path(text) or traversable else "blocked",
            "reason": text,
        })
    #生成单侧通行和地形信息

    @classmethod
    def _build_box_fact(cls, text):
        height = cls._extract_height_meters(text) or 0.0
        return {
            "name": "box",
            "side": "left" if "左" in text else "right" if "右" in text else "unknown",
            "height_m": height,
            "movable": True,
            "usable_as_step": height <= cls.MAX_CLIMB_HEIGHT_M,
            "description": text,
        }
    #从障碍物描述中提取箱子事实

    @classmethod
    def _build_summary(cls, left_text, right_text, front_text):
        parts = []
        cls._append_side_summary(parts, "左侧", left_text)
        cls._append_side_summary(parts, "右侧", right_text)
        if not parts and front_text != "unknown":
            parts.append(front_text)
        return "，".join(parts) if parts else "环境信息不足"
    #生成给LLM阅读的环境摘要

    @classmethod
    def _append_side_summary(cls, parts, label, text):
        height = cls._extract_height_meters(text)
        if height is not None:
            parts.append(f"{label}存在约{height:g}米高台")
        elif cls._is_clear_path(text):
            parts.append(f"{label}可通行")
        elif text != "unknown":
            parts.append(text if text.startswith(label) else f"{label}{text}")
    #把单侧描述压缩成一句话

    @classmethod
    def _parse_structured_output(cls, content):
        text = response_to_text(content)
        if text.startswith("```"):
            lines = text.splitlines()
            text = "\n".join(lines[1:-1] if lines[-1].strip() == "```" else lines[1:])
        payload = json.loads(text)
        return cls._normalize_payload(payload if isinstance(payload, dict) else {})
    #解析并标准化VLM输出

    @classmethod
    def _normalize_payload(cls, payload):
        result = dict(cls.DEFAULT_OUTPUT)
        result["ground"] = cls._normalize_text(payload.get("ground"))
        result["left_side"] = cls._normalize_text(payload.get("left_side"))
        result["right_side"] = cls._normalize_text(payload.get("right_side"))
        result["front_area"] = cls._normalize_text(payload.get("front_area"))
        result["obstacles"] = cls._normalize_list(payload.get("obstacles"))
        result["suspected_height_diff"] = bool(payload.get("suspected_height_diff"))
        result["uncertainties"] = cls._normalize_list(payload.get("uncertainties"))
        return result
    #补齐VLM输出字段，避免缺字段影响Planner

    @staticmethod
    def _normalize_text(value):
        text = str(value or "").strip()
        return text if text else "unknown"
    #把空文本转成unknown

    @classmethod
    def _normalize_list(cls, value):
        if isinstance(value, list):
            return [cls._normalize_text(item) for item in value if cls._normalize_text(item) != "unknown"]
        text = cls._normalize_text(value)
        return [] if text == "unknown" else [text]
    #把字段统一成字符串列表

    @staticmethod
    def _extract_height_meters(text):
        match = re.search(r"(\d+(?:\.\d+)?)\s*米", text)
        return float(match.group(1)) if match else None
    #从中文描述里提取米制高度

    @staticmethod
    def _is_clear_path(text):
        return any(keyword in text for keyword in ("无障碍", "可通行", "畅通", "开阔", "平地"))
    #判断描述是否表示可通行路线

    @staticmethod
    def _is_wall_like(text):
        return any(keyword in text for keyword in ("墙", "墙体", "围墙", "垂直结构"))
    #判断描述是否更像不可通行墙体
