import re

ACTION_TOOLS = {"nav", "walk_skill", "push", "climb"}

PRE_OBSERVE_INSTRUCTION = (
    "运行时规划约束：当前任务还没有完成环境观察。本轮只能调用 observe，"
    "不要提前调用 nav、push、climb 或 walk_skill。"
)

POST_OBSERVE_QUEUE_INSTRUCTION = (
    "运行时规划约束：你已经拿到 observe 的结构化结果。"
    "如果任务需要多个动作步骤，本轮必须一次性返回完整动作工具队列，"
    "例如 nav -> climb -> nav 要全部放在同一轮 tool_calls 中。"
    "正文规划也必须按完整队列描述，不要写“现在先调用第一步”“下一步再调用”。"
    "执行器会连续执行队列，只有失败、超时或状态不确定时才会重新交给上层模型规划。"
)


def has_action_tool(tool_calls):
    return any(tc["name"] in ACTION_TOOLS for tc in tool_calls)


def planning_messages(base_messages, observed):
    instruction = POST_OBSERVE_QUEUE_INSTRUCTION if observed else PRE_OBSERVE_INSTRUCTION
    return [*base_messages, {"role": "user", "content": instruction}]


def incomplete_tool_batch(content, tool_calls):
    if not has_action_tool(tool_calls):
        return False
    text = str(content or "")
    numbered_steps = re.findall(r"(?m)^\s*(?:\d+[.、)]|第[一二三四五六七八九十]+步)", text)
    if len(numbered_steps) > len(tool_calls):
        return True
    if len([tc for tc in tool_calls if tc["name"] in ACTION_TOOLS]) >= 2:
        return False
    return bool(
        re.search(r"(然后|随后|接下来|下一步|最后|第二步|第三步)", text)
        and re.search(r"(nav|climb|push|walk_skill|导航|攀爬|推箱|行走)", text)
    )


def should_replan_after_batch(plan_results, tool_calls):
    if any(item.get("signal") == "FAILURE" for item in plan_results or []):
        return True
    if not has_action_tool(tool_calls):
        return True
    return False
