import os
import json
import sys
from dotenv import load_dotenv
from openai import OpenAI
import pyarrow as pa

try:
    import yaml
except ImportError:
    raise ImportError("pip install pyyaml")

from dora import Node

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
            text = item["text"]
            # Using safe template replacement
            if "{user_input}" in text:
                text = text.replace("{user_input}", kwargs.get("user_input", ""))
            new_msg["content"].append({"type": "text", "text": text})
        rendered.append(new_msg)
    return rendered


# ---------- Main ----------
if __name__ == "__main__":

    # Initialize a real Dora node
    node = Node()
    print("LLM_TRANSLATOR: Node initialized.")

    # Test different user inputs
    test_inputs = [
        "Go forward 50cm",
        "turn right and move 1 meter",
        "Go to the table left 50cm",
        "go back 20cm",
        "Pick up the cup", # This will be unhandled by sim, but will be sent
    ]
    
    for user_input in test_inputs:
        print(f"\n{'='*60}")
        print(f"LLM_TRANSLATOR: Processing Input: {user_input}")
        print('='*60)
        
        try:
            # 1. Load + render prompt for Dora
            messages = load_prompt(
                "task-to-dora-action",
                "demo2dora.yaml"
            )
            messages = render_messages(messages, user_input=user_input)

            # 2. Call LLM
            completion = client.chat.completions.create(
                model="qwen-plus",
                messages=messages
            )

            raw_output = completion.choices[0].message.content

            # 3. Parse JSON
            command = {}
            try:
                command = json.loads(raw_output.strip())
            except json.JSONDecodeError:
                import re
                json_match = re.search(r'\{.*\}', raw_output, re.DOTALL)
                if json_match:
                    command = json.loads(json_match.group())
                else:
                    print("LLM_TRANSLATOR: LLM output was not valid JSON.")
                    continue # Skip this input

            # 4. Dispatch command to Dora
            action_name = command.get("action")
            if not action_name:
                print("LLM_TRANSLATOR: Parsed command is missing 'action' field.")
                continue

            print(f"LLM_TRANSLATOR: Parsed Command: {command}")
            
            # Use PyArrow to serialize the data for Dora
            # The data must be in a list/array format.
            arrow_data = pa.array([command])
            node.send_output("command", arrow_data)
            
            print(f"LLM_TRANSLATOR: Sent '{action_name}' command to Dora.")

            # Add a delay to see the simulation happen sequentially
            import time
            time.sleep(5)


        except Exception as e:
            print(f"\n❌ LLM_TRANSLATOR: Error: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()

    print("LLM_TRANSLATOR: Finished all test inputs. Stopping node.")
