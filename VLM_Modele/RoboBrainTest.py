from openai import OpenAI

# 配置客户端
client = OpenAI(
    base_url="http://192.168.0.xxx:8000/v1",
    #注意改成已经部署好的电脑IP地址.......端口号也是提前设置的
    api_key="not-needed"  # vLLM不需要API密钥
)

# 聊天模式
response = client.chat.completions.create(
    model="robobrain",
    messages=[
        # {"role": "system", "content": "You are a creative writer."},
        {"role": "user", "content": "who are you?"}
    ],
    # max_tokens=200,
    # temperature=0.7
)

print("\nChat response:", response.choices[0].message.content)
