# -*- coding: utf-8 -*-
# ChaseBot_Project/llm_agent.py
import sys
import os
import json
import yaml
from openai import OpenAI
from dotenv import load_dotenv
import pyarrow as pa
from dora import Node
import time

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# --- Helper Functions ---

def load_prompt_template(file_path):
    """Loads the prompt template from a YAML file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"AGENT ERROR: Prompt file not found at {file_path}")
        sys.exit(1)
    except Exception as e:
        print(f"AGENT ERROR: Failed to load or parse prompt file: {e}")
        sys.exit(1)

def render_prompt(template, context):
    """Renders the user prompt with the given context."""
    user_prompt = template['user_prompt']
    for key, value in context.items():
        user_prompt = user_prompt.replace(f"{{{{ {key} }}}}", str(value))
    return user_prompt

# --- Main Agent Class ---

class LLMAgent:
    """
    The LLM Agent that observes the game state, thinks, and acts.
    """
    def __init__(self):
        # Load API Key from the parent project's .env file
        dotenv_path = os.path.join(sys.path[0], '..', 'VLM_Modele', '.env')
        load_dotenv(dotenv_path=dotenv_path)
        api_key = os.getenv("Test_API_KEY")
        if not api_key:
            print("AGENT ERROR: Test_API_KEY not found. Please check your .env file.")
            sys.exit(1)

        self.client = OpenAI(
            api_key=api_key,
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        
        # Load prompts
        prompt_path = os.path.join(os.path.dirname(__file__), 'LLM_prompts', 'chase_bot_prompt.yaml')
        self.prompt_template = load_prompt_template(prompt_path)
        
        self.node = Node("llm_agent")
        self.last_action_time = 0
        self.action_cooldown = 2 # seconds

    def get_llm_decision(self, game_state):
        """
        Queries the LLM for the next action based on the current game state.
        """
        context = {
            'player_x': game_state['player_bot']['x'],
            'player_y': game_state['player_bot']['y'],
            'player_angle': game_state['player_bot']['angle'],
            'target_x': game_state['target_bot']['x'],
            'target_y': game_state['target_bot']['y'],
        }
        
        rendered_user_prompt = render_prompt(self.prompt_template, context)
        
        print("\n" + "="*20 + " AGENT: Asking LLM " + "="*20)
        print(f"AGENT: Current State:\n{rendered_user_prompt}")

        try:
            completion = self.client.chat.completions.create(
                model="qwen-plus",
                messages=[
                    {"role": "system", "content": self.prompt_template['system_prompt']},
                    {"role": "user", "content": rendered_user_prompt},
                ],
                temperature=0.7,
            )
            response_text = completion.choices[0].message.content
            
            # Clean up the response to extract pure JSON
            json_part = response_text[response_text.find('{'):response_text.rfind('}')+1]
            decision = json.loads(json_part)
            
            print(f"AGENT: LLM Response (raw): {response_text}")
            print(f"AGENT: LLM Decision (parsed): {decision}")
            return decision

        except json.JSONDecodeError:
            print(f"AGENT ERROR: Failed to decode JSON from LLM response: {response_text}")
        except Exception as e:
            print(f"AGENT ERROR: An error occurred while querying the LLM: {e}")
        
        return None

    def run(self):
        """
        Main loop for the agent node.
        """
        print("AGENT: LLM Agent is running. Waiting for game state...")
        
        while True:
            try:
                dora_event = self.node.next()
                if not dora_event:
                    break # End of stream
                
                if dora_event["type"] == "INPUT":
                    if dora_event["id"] == "game_state":
                        # Throttle actions to avoid spamming the LLM
                        current_time = time.time()
                        if current_time - self.last_action_time < self.action_cooldown:
                            continue

                        self.last_action_time = current_time
                        
                        game_state = dora_event["value"][0].as_py()
                        
                        llm_decision = self.get_llm_decision(game_state)

                        if llm_decision:
                            self.node.send_output("command", pa.array([llm_decision]))


            except Exception as e:
                print(f"AGENT ERROR: An error occurred in the main loop: {e}")
                break

if __name__ == "__main__":
    agent = LLMAgent()
    agent.run()
