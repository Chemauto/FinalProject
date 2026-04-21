import os
import yaml
from pathlib import Path
from dotenv import load_dotenv

for var in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "all_proxy", "no_proxy", "NO_PROXY"):
    os.environ.pop(var, None)
#删除代理，防止proxy导致报错

load_dotenv()
prompt_path = Path(__file__).parent / "prompts" / "planner_prompt.yaml"
with open(prompt_path, "r", encoding="utf-8") as f:
    prompt = yaml.safe_load(f)
#加载提示词

def chat(messages):
    from openai import OpenAI

    client = OpenAI(
        api_key=os.getenv("MODEL_API_KEY"),
        base_url=os.getenv("MODEL_BASE_URL"),
    )
    #创建模型客户端

    completion = client.chat.completions.create(
        model=prompt["model"],
        messages=messages,
    )
    return completion.choices[0].message.content
#调用模型并返回文本

def stream_chat(messages):
    from openai import OpenAI

    client = OpenAI(
        api_key=os.getenv("MODEL_API_KEY"),
        base_url=os.getenv("MODEL_BASE_URL"),
    )
    #创建模型客户端

    stream = client.chat.completions.create(
        model=prompt["model"],
        messages=messages,
        stream=True,
    )
    for chunk in stream:
        text = chunk.choices[0].delta.content
        if text:
            yield text
#流式调用模型，逐段返回文本

if __name__ == "__main__":
    user_input = input("请输入你的问题：").strip() or prompt["user_prompt"]
    messages = [
        {"role": "system", "content": prompt["system_prompt"]},
        {"role": "user", "content": user_input},
    ]
    print(chat(messages))
    #人工输入，如果直接回车就使用yaml里的默认用户提示词
