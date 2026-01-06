# -*- coding: utf-8 -*-
# ROS_Module/ros_module/nodes/llm_receiver_node.py
import sys
import os
import json
from dotenv import load_dotenv
from openai import OpenAI
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

try:
    import yaml
except ImportError:
    raise ImportError("pip install pyyaml")

# ---------- Env ----------
# Adjust the path to the .env file, assuming it's in the VLM_Modele directory
dotenv_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'VLM_Modele', '.env')
load_dotenv(dotenv_path=dotenv_path)
API_KEY = os.getenv("Test_API_KEY")
if not API_KEY:
    raise ValueError("Missing Test_API_KEY")


# ---------- LLM Client ----------
class LLMClient:
    """
    A client for interacting with the LLM.
    """

    def __init__(self):
        """
        Initializes the LLMClient.
        """
        self.client = OpenAI(
            api_key=API_KEY,
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        self.base_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'VLM_Modele')

    def load_prompt(self, prompt_id, file):
        """
        Loads a prompt from a YAML file.

        Args:
            prompt_id (str): The ID of the prompt to load.
            file (str): The name of the YAML file.

        Returns:
            list: The loaded prompt messages.
        """
        path = os.path.join(self.base_dir, "LLM_prompts", "Basic_prompts", file)
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or []
        for entry in data:
            if entry.get("id") == prompt_id:
                return entry["messages"]
        raise ValueError(f"Prompt id not found: {prompt_id}")

    def render_messages(self, messages, **kwargs):
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

    def get_completion(self, messages):
        """
        Gets a completion from the LLM.

        Args:
            messages (list): The messages to send to the LLM.

        Returns:
            str: The completion from the LLM.
        """
        completion = self.client.chat.completions.create(
            model="qwen-plus", messages=messages
        )
        return completion.choices[0].message.content


# ---------- ROS2 Node ----------
class LLMReceiverNode(Node):
    """
    A ROS2 node that receives user commands, processes them with an LLM, and publishes the resulting robot commands.
    """

    def __init__(self, llm_client):
        """
        Initializes the LLMReceiverNode.

        Args:
            llm_client (LLMClient): The LLM client to use.
        """
        super().__init__('llm_receiver_node')
        self.llm_client = llm_client
        self.subscription = self.create_subscription(
            String,
            'user_command',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'robot_command', 10)
        self.get_logger().info('LLM Receiver Node is running. Waiting for user commands...')

    def listener_callback(self, msg):
        """
        Callback function for the user command subscriber.

        Args:
            msg (std_msgs.msg.String): The received message.
        """
        user_input = msg.data
        self.get_logger().info(f"Received user command: '{user_input}'")

        try:
            # 1. Load + render prompt
            # Using demo2ros.yaml as it might be more suitable for ROS
            messages = self.llm_client.load_prompt("task-to-ros-action", "demo2ros.yaml")
            messages = self.llm_client.render_messages(messages, user_input=user_input)

            # 2. Call LLM
            self.get_logger().info("Calling LLM...")
            raw_output = self.llm_client.get_completion(messages)

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
                    self.get_logger().warn("LLM output was not valid JSON.")
                    self.get_logger().warn(f"   Raw Output: {raw_output}")
                    return

            # 4. Dispatch command to Simulator via ROS2
            action_name = command.get("action")
            if not action_name:
                self.get_logger().warn("Parsed command is missing 'action' field.")
                return

            self.get_logger().info(f"Parsed Command: {command}")
            
            # Publish the command as a JSON string
            command_msg = String()
            command_msg.data = json.dumps(command)
            self.publisher_.publish(command_msg)
            self.get_logger().info(f"Sent '{action_name}' command to simulator.")

        except Exception as e:
            self.get_logger().error(f"An error occurred: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()


def main(args=None):
    """
    Main function for the LLM receiver node.
    """
    rclpy.init(args=args)
    llm_client = LLMClient()
    llm_receiver_node = LLMReceiverNode(llm_client)
    rclpy.spin(llm_receiver_node)
    llm_receiver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
