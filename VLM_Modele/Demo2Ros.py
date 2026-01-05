import os
import json
from dotenv import load_dotenv
from openai import OpenAI

try:
    import yaml
except ImportError:
    raise ImportError("pip install pyyaml")


# ---------- Env ----------
load_dotenv()
API_KEY = os.getenv("Test_API_KEY")
if not API_KEY:
    raise ValueError("Missing Test_API_KEY")


# ---------- OpenAI Client ----------
client = OpenAI(
    api_key=API_KEY,
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
)


BASE_DIR = os.path.dirname(__file__)


# ---------- Prompt Loader ----------
def load_prompt(prompt_id, file):
    path = os.path.join(BASE_DIR, "LLM_prompts", "Basic_prompts", file)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or []

    for entry in data:
        if entry.get("id") == prompt_id:
            return entry["messages"]

    raise ValueError(f"Prompt id not found: {prompt_id}")


def render_messages(messages, **kwargs):
    rendered = []
    for msg in messages:
        new_msg = {"role": msg["role"], "content": []}
        for item in msg["content"]:
            text = item["text"].format(**kwargs)
            new_msg["content"].append({"type": "text", "text": text})
        rendered.append(new_msg)
    return rendered


# ---------- Main ----------
if __name__ == "__main__":

    user_input = "Go to the table"

    # 1. Load + render prompt
    messages = load_prompt(
        "task-to-ros2-action",
        "demo2ros.yaml"
    )
    messages = render_messages(messages, user_input=user_input)

    # 2. Call LLM
    completion = client.chat.completions.create(
        model="qwen-plus",
        messages=messages
    )

    raw_output = completion.choices[0].message.content
    print("LLM RAW OUTPUT:")
    print(raw_output)

    # 3. Parse JSON
    try:
        command = json.loads(raw_output)
    except json.JSONDecodeError as e:
        raise RuntimeError("LLM output is not valid JSON") from e

    # 4. Dispatch to ROS2
    action_name = command["action"]
    parameters = command.get("parameters", {})

    print("Parsed Command:", command)

    if action_name == "navigate":
        from ros2_actions.navigate_client import send_navigate_goal
        send_navigate_goal(parameters)

    else:
        print(f"Unknown action: {action_name}")
