import os
from base64 import b64encode
from pathlib import Path


def load_api_key(root):
    if os.getenv("MODEL_API_KEY"):
        return os.getenv("MODEL_API_KEY")
    if os.getenv("Test_API_KEY"):
        return os.getenv("Test_API_KEY")

    env_file = Path(root).parent / ".env"
    if env_file.is_file():
        for line in env_file.read_text(encoding="utf-8").splitlines():
            if "=" not in line or line.strip().startswith("#"):
                continue
            name, value = line.split("=", 1)
            if name.strip() in {"MODEL_API_KEY", "Test_API_KEY"}:
                return value.strip().strip('"').strip("'")
    raise ValueError("未找到VLM API Key，请设置MODEL_API_KEY")
#读取VLM API Key，优先环境变量，其次项目.env


def to_data_url(image_file):
    image_file = Path(image_file)
    mime = {
        ".jpg": "image/jpeg",
        ".jpeg": "image/jpeg",
        ".png": "image/png",
        ".bmp": "image/bmp",
        ".webp": "image/webp",
    }.get(image_file.suffix.lower(), "image/png")
    encoded = b64encode(image_file.read_bytes()).decode("utf-8")
    return f"data:{mime};base64,{encoded}"
#把本地图片转成OpenAI兼容接口需要的data URL


def response_to_text(content):
    if isinstance(content, str):
        return content.strip()
    return "".join(block.get("text", "") for block in content or []).strip()
#兼容字符串和分段content两种模型返回格式
