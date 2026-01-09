import os
from openai import OpenAI
from dotenv import load_dotenv
import json


# 加载 .env 文件

project_root = os.path.abspath(os.path.join(os.getcwd(), '..'))
dotenv_path = os.path.join(project_root, '.env')
load_dotenv(dotenv_path=dotenv_path)
API_KEY = os.getenv("Test_API_KEY")
if not API_KEY:
    raise ValueError("请在 .env 文件中设置 Test_API_KEY")

#配置密钥，也可以直接使用export导入
client = OpenAI(
    # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx"
    # api_key=os.getenv("Test_API_KEY"),
    api_key=API_KEY,
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
)
# Load prompt from YAML (if available) and send to model
# Requires PyYAML to read prompts: `pip install pyyaml`
try:
    import yaml
except Exception:
    yaml = None


def load_prompt(prompt_id='ask-who'):
    
    #获取yaml文件路径
    prompts_path = os.path.join(os.path.dirname(__file__), "LLM_prompts", "Basic_prompts", "demoStart.yaml")
    
    #1.默认初始化，这个就只是为了测试
    # Fallback prompt if YAML not available or prompt not found
    # fallback = [{"role": "user", "content": [{"type": "text", "text": "who are u?"}]}]
    # if not yaml:
        # return fallback
    #2.零值初始化
    fallback=None

    #读取yaml文件内容
    try:
        with open(prompts_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or []
    except FileNotFoundError:
        return fallback

    #根据id得到相应的内容message
    for entry in data:
        if entry.get('id') == prompt_id:
            return entry.get('messages', fallback)
        
    return fallback
    #调用函数
messages = load_prompt('ask-who')

completion = client.chat.completions.create(
    model="qwen-vl-plus",  # 可按需更换模型名称
    messages=messages,
    #规范输出的格式
    response_format={"type": "json_object"}
)
#输出json格式
response_content = completion.choices[0].message.content
try:
    response_json = json.loads(response_content)
    print("JSON输出:", response_json)

except json.JSONDecodeError as e:
    print("解析JSON失败:", e)
    print("原始输出:", response_content)
# print(completion.model_dump_json())