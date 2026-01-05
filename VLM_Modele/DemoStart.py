import os
from openai import OpenAI

client = OpenAI(
    # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx"
    api_key=os.getenv("Test_API_KEY"),
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
)
# Load prompt from YAML (if available) and send to model
# Requires PyYAML to read prompts: `pip install pyyaml`
try:
    import yaml
except Exception:
    yaml = None


def load_prompt(prompt_id='ask-who'):
    prompts_path = os.path.join(os.path.dirname(__file__), "LLM_prompts", "Basic_prompts", "basc.yaml")
    # Fallback prompt if YAML not available or prompt not found
    fallback = [{"role": "user", "content": [{"type": "text", "text": "who are u?"}]}]
    if not yaml:
        return fallback

    try:
        with open(prompts_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or []
    except FileNotFoundError:
        return fallback

    for entry in data:
        if entry.get('id') == prompt_id:
            return entry.get('messages', fallback)

    return fallback

messages = load_prompt('ask-who')

completion = client.chat.completions.create(
    model="qwen-vl-plus",  # 可按需更换模型名称
    messages=messages
)

messages = completion.choices[0].message.content
print(messages)
# print(completion.model_dump_json())