from __future__ import annotations
"""VLM 通用工具。"""

import os
from base64 import b64encode
from pathlib import Path


def load_api_key(root: Path) -> str:
    """优先读环境变量，其次读项目根目录 .env。"""
    if os.getenv("Test_API_KEY"):
        return os.getenv("Test_API_KEY")

    env_file = root.parent / ".env"
    if env_file.is_file():
        for line in env_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            name, value = line.split("=", 1)
            if name.strip() == "Test_API_KEY":
                return value.strip().strip('"').strip("'")

    raise ValueError("未找到 VLM API Key，请在项目根目录 .env 中设置 Test_API_KEY")


def to_data_url(image_file: Path) -> str:
    """把本地图片编码成 data URL。"""
    mime = {
        ".jpg": "image/jpeg",
        ".jpeg": "image/jpeg",
        ".png": "image/png",
        ".bmp": "image/bmp",
        ".webp": "image/webp",
    }.get(image_file.suffix.lower(), "image/png")
    encoded = b64encode(image_file.read_bytes()).decode("utf-8")
    return f"data:{mime};base64,{encoded}"


def response_to_text(content) -> str:
    """兼容字符串和分段消息两种返回格式。"""
    if isinstance(content, str):
        return content.strip()
    return "".join(block.get("text", "") for block in content or []).strip()
