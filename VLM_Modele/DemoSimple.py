import os
from openai import OpenAI
from dotenv import load_dotenv

# # 加载 .env 文件
# load_dotenv()

# api_key = os.getenv("Test_API_KEY")
# if not api_key:
#     raise ValueError("请在 .env 文件中设置 Test_API_KEY")

client = OpenAI(
    # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx"
    #1.
    # api_key=os.getenv("Test_API_KEY"),#通过环境变量设置完成
    #export Test_API_KEY=xxxxxxxxx
    #2.
    #直接导入下面的密钥
    # api_key="xxxxxxxxxxxxxxxxxxx",
    #3.
    #通过.env文件设置
    api_key="not-needed"
    base_url="https://192.168.0.233:8000/v1",
)
completion = client.chat.completions.create(
    model="robobrain",  # 此处以qwen-vl-plus为例，可按需更换模型名称。模型列表：https://help.aliyun.com/zh/model-studio/getting-started/models
    messages=[{"role": "user","content": [
            {"type": "image_url",
             "image_url": {"url": "https://dashscope.oss-cn-beijing.aliyuncs.com/images/dog_and_girl.jpeg"}},
            {"type": "text", "text": "这是什么"},
            ]}]
    )
messages=completion.choices[0].message.content
print(messages)
# print(completion.model_dump_json())