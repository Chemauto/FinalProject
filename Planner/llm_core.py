import os
import json
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

def chat(messages, tools=None):
    from openai import OpenAI

    client = OpenAI(
        api_key=os.getenv("MODEL_API_KEY"),
        base_url=os.getenv("MODEL_BASE_URL"),
    )
    #创建模型客户端

    kwargs = {"model": prompt["model"], "messages": messages}
    if tools:
        kwargs["tools"] = tools
        kwargs["tool_choice"] = "auto"
    #有tools时传给API

    completion = client.chat.completions.create(**kwargs)
    msg = completion.choices[0].message
    if msg.tool_calls:
        return {"type": "tool_calls", "tool_calls": [
            {"name": tc.function.name, "args": json.loads(tc.function.arguments)}
            for tc in msg.tool_calls
        ]}
    return msg.content
#调用模型，有tool_calls返回工具调用列表，否则返回文本

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

def make_plan(messages):
    from Executor.tools import get_tool_definitions
    result = chat(messages, tools=get_tool_definitions())
    if isinstance(result, dict) and result["type"] == "tool_calls":
        return {"type": "plan", "tool_calls": result["tool_calls"]}
    return {"type": "text", "content": result}
#带工具定义调LLM，有tool_calls返回计划，否则返回文本

if __name__ == "__main__":
    user_input = input("请输入你的问题：").strip() or prompt["user_prompt"]
    messages = [
        {"role": "system", "content": prompt["system_prompt"]},
        {"role": "user", "content": user_input},
    ]
    print(chat(messages))
    #人工输入，如果直接回车就使用yaml里的默认用户提示词
