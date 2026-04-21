from Planner.llm_core import prompt

messages = [{"role": "system", "content": prompt["system_prompt"]}]
#保存上下文

def reset():
    messages.clear()
    messages.append({"role": "system", "content": prompt["system_prompt"]})
#重置上下文
