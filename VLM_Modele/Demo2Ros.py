import os
import json
import sys
from dotenv import load_dotenv
from openai import OpenAI

try:
    import yaml
except ImportError:
    raise ImportError("pip install pyyaml")


# ---------- Env ----------
project_root = os.path.abspath(os.path.join(os.getcwd(), '..'))
dotenv_path = os.path.join(project_root, '.env')
load_dotenv(dotenv_path=dotenv_path)
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
            text = item["text"]
            # 使用安全的模板替换
            if "{user_input}" in text:
                text = text.replace("{user_input}", kwargs.get("user_input", ""))
            new_msg["content"].append({"type": "text", "text": text})
        rendered.append(new_msg)
    return rendered


# ---------- 模拟 ROS2 动作函数 ----------
def mock_send_navigate_goal(parameters):
    """
    模拟导航动作，支持位置、方向和距离
    """
    print("\n" + "="*50)
    print("🎯 模拟 ROS2 导航动作执行")
    print("="*50)
    print(f"动作类型: navigate")

    location = parameters.get('location', '未知位置')
    direction = parameters.get('direction', None)
    distance = parameters.get('distance', None)

    print(f"目标位置: {location}")

    # 如果有方向和距离参数，计算相对位置
    if direction and distance:
        direction_cn = {
            'front': '前方',
            'back': '后方',
            'left': '左侧',
            'right': '右侧'
        }.get(direction.lower(), direction)

        print(f"相对方向: {direction_cn}")
        print(f"相对距离: {distance}")
        print(f"\n导航路径描述: 前往 {location}，然后向{direction_cn}移动 {distance}")
        print("正在执行导航...")
        print("✅ 导航完成!")
    else:
        print(f"\n导航路径描述: 直接前往 {location}")
        print("正在前往目标位置...")
        print("✅ 导航完成!")

    print("="*50 + "\n")


def mock_send_pick_goal(parameters):
    """
    模拟抓取动作
    """
    print("\n" + "="*50)
    print("🤖 模拟 ROS2 抓取动作执行")
    print("="*50)
    print(f"动作类型: pick")
    print(f"抓取对象: {parameters.get('object', '未知物体')}")
    print(f"详细参数: {parameters}")
    print("正在执行抓取...")
    print("✅ 抓取完成!")
    print("="*50 + "\n")


def mock_send_place_goal(parameters):
    """
    模拟放置动作
    """
    print("\n" + "="*50)
    print("📦 模拟 ROS2 放置动作执行")
    print("="*50)
    print(f"动作类型: place")
    print(f"放置位置: {parameters.get('location', '未知位置')}")
    print(f"详细参数: {parameters}")
    print("正在执行放置...")
    print("✅ 放置完成!")
    print("="*50 + "\n")


# ---------- Main ----------
if __name__ == "__main__":

    # 测试不同的用户输入
    test_inputs = [
        "Navigate to the kitchen",  # 基础导航
        "Navigate to the kitchen front 30cm",  # 带方向和距离
        "Go to the table left 50cm",  # 左侧移动
        "去床后面1米处",  # 后方移动，使用米
        "Pick up the cup",  # 抓取物体
        "Place the book on the shelf"  # 放置物体
    ]
    
    for user_input in test_inputs:
        print(f"\n{'='*60}")
        print(f"📝 用户输入: {user_input}")
        print('='*60)
        
        try:
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
            print("\n🤖 LLM 原始输出:")
            print(raw_output)

            # 3. Parse JSON
            try:
                command = json.loads(raw_output.strip())
            except json.JSONDecodeError as e:
                print(f"⚠️  JSON 解析错误: {e}")
                # 尝试提取 JSON 部分
                import re
                json_match = re.search(r'\{.*\}', raw_output, re.DOTALL)
                if json_match:
                    command = json.loads(json_match.group())
                else:
                    raise RuntimeError("LLM output is not valid JSON") from e

            # 4. 根据动作类型分发
            action_name = command["action"]
            parameters = command.get("parameters", {})

            print(f"\n✅ 解析后的命令:")
            print(f"   动作: {action_name}")
            print(f"   参数: {parameters}")

            # 使用模拟函数
            if action_name == "navigate":
                mock_send_navigate_goal(parameters)
            elif action_name == "pick":
                mock_send_pick_goal(parameters)
            elif action_name == "place":
                mock_send_place_goal(parameters)
            else:
                print(f"❌ 未知动作: {action_name}")

        except Exception as e:
            print(f"\n❌ 错误: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()