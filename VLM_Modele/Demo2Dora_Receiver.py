# -*- coding: utf-8 -*-
# VLM_Modele/5Demo2Dora_Receiver.py
import sys
import os
import json
from dotenv import load_dotenv
from openai import OpenAI
import pyarrow as pa

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

try:
    import yaml
except ImportError:
    raise ImportError("pip install pyyaml")

from dora import Node

# ---------- Env ----------
# Dora runs scripts from the 'Dora_Module' directory. To find the .env file in the project root,
# we get the current working directory, go one level up, and then join with '.env'.
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
# 配置客户端


BASE_DIR = os.path.dirname(__file__)


# ---------- Prompt Loader ----------
def load_prompt(prompt_id, file):
    """
    Loads a prompt from a YAML file.

    Args:
        prompt_id (str): The ID of the prompt to load.
        file (str): The name of the YAML file.

    Returns:
        list: The loaded prompt messages.
    """
    path = os.path.join(BASE_DIR, "LLM_prompts", "Basic_prompts", file)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or []
    for entry in data:
        if entry.get("id") == prompt_id:
            return entry["messages"]
    raise ValueError(f"Prompt id not found: {prompt_id}")


def render_messages(messages, **kwargs):
    """
    Renders the messages with the given keyword arguments.

    Args:
        messages (list): The messages to render.
        **kwargs: The keyword arguments to use for rendering.

    Returns:
        list: The rendered messages.
    """
    rendered = []
    for msg in messages:
        new_msg = {"role": msg["role"], "content": []}
        for item in msg["content"]:
            text = item["text"].replace("{user_input}", kwargs.get("user_input", ""))
            new_msg["content"].append({"type": "text", "text": text})
        rendered.append(new_msg)
    return rendered


# ---------- Main Dora Node Loop ----------
if __name__ == "__main__":
    node = Node()
    print("LLM_RECEIVER: Node initialized. Waiting for user commands...")

    for event in node:
        if event["type"] != "INPUT":
            continue
        
        # We expect the input to be a PyArrow array with one string element.
        user_input = event["value"][0].as_py()
        print("\n" + "="*60)
        print(f"LLM_RECEIVER: Received command: '{user_input}'")

        try:
            # 1. Load + render prompt
            messages = load_prompt("task-to-dora-action", "demo2dora.yaml")
            messages = render_messages(messages, user_input=user_input)

            # 2. Call LLM
            print("LLM_RECEIVER: Calling LLM...")
            completion = client.chat.completions.create(
                model="qwen-plus", messages=messages
                # model="robobrain", messages=messages
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
                    print("LLM_RECEIVER: WARNING: LLM output was not valid JSON.")
                    print(f"   Raw Output: {raw_output}")
                    continue

            # 4. Handle both old format (single action) and new format (actions array)
            actions_to_send = []

            # Check if new format with "actions" array
            if "actions" in command and isinstance(command["actions"], list):
                actions_to_send = command["actions"]
                print(f"LLM_RECEIVER: Parsed {len(actions_to_send)} actions")
            # Check if old format with single "action"
            elif "action" in command:
                actions_to_send = [command]
                print("LLM_RECEIVER: Parsed single action (legacy format)")
            else:
                print("LLM_RECEIVER: WARNING: Parsed command is missing 'action' or 'actions' field.")
                print(f"   Command: {command}")
                continue

            # 5. Dispatch each action to Simulator via Dora in sequence
            for idx, action in enumerate(actions_to_send, 1):
                action_name = action.get("action")
                if not action_name:
                    print(f"LLM_RECEIVER: WARNING: Action {idx} is missing 'action' field.")
                    continue

                print(f"LLM_RECEIVER: [{idx}/{len(actions_to_send)}] Sending {action}: {action.get('parameters', {})}")
                arrow_data = pa.array([action])
                node.send_output("command", arrow_data)
                print(f"LLM_RECEIVER: Sent '{action_name}' command to simulator.")

        except Exception as e:
            print(f"\nLLM_RECEIVER: ERROR: An error occurred: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
