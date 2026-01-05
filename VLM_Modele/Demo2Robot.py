import os
from dotenv import load_dotenv
from openai import OpenAI

# ---------- 可选依赖 ----------
try:
    import yaml
except ImportError:
    raise ImportError("请先安装 PyYAML: pip install pyyaml")


# ---------- 环境变量 ----------
load_dotenv()

API_KEY = os.getenv("Test_API_KEY")
if not API_KEY:
    raise ValueError("请在 .env 文件中设置 Test_API_KEY")


# ---------- OpenAI / DashScope Client ----------
client = OpenAI(
    api_key=API_KEY,
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
)


# ---------- Prompt Loader ----------
BASE_DIR = os.path.dirname(__file__)


def load_prompt(prompt_id: str, file: str = "new.yaml"):
    """
    Load prompt messages from YAML by id
    """
    prompts_path = os.path.join(
        BASE_DIR, "LLM_prompts", "Basic_prompts", file
    )

    fallback = [{
        "role": "user",
        "content": [{"type": "text", "text": "Hello"}]
    }]

    try:
        with open(prompts_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or []
    except FileNotFoundError:
        print(f"[WARN] Prompt file not found: {prompts_path}")
        return fallback

    for entry in data:
        if entry.get("id") == prompt_id:
            return entry.get("messages", fallback)

    print(f"[WARN] Prompt id not found: {prompt_id}")
    return fallback


# ---------- Prompt Renderer ----------
def render_messages(messages, **kwargs):
    """
    Replace template variables in prompt messages
    """
    rendered = []
    for msg in messages:
        new_msg = {
            "role": msg["role"],
            "content": []
        }
        for item in msg["content"]:
            if item["type"] == "text":
                text = item["text"].format(**kwargs)
                new_msg["content"].append({
                    "type": "text",
                    "text": text
                })
        rendered.append(new_msg)
    return rendered


# ---------- Main ----------
if __name__ == "__main__":

    # ========== 选择 Prompt ==========
    PROMPT_ID = "robot-task-demo"
    USER_INPUT = "Go to the table and pick up the cup"

    # 1. Load prompt
    messages = load_prompt(PROMPT_ID)

    # 2. Render template
    messages = render_messages(
        messages,
        user_input=USER_INPUT
    )

    # 3. Call LLM
    completion = client.chat.completions.create(
        model="qwen-plus",   # 纯文本模型即可
        messages=messages
    )

    # 4. Print result
    reply = completion.choices[0].message.content
    print("=== LLM Output ===")
    print(reply)
