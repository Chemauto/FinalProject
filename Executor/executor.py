from Executor.tools import call_tool
from Executor.state import fmt_robot, fmt_box

def run_plan(tool_calls, emit):
    emit("plan", f"共{len(tool_calls)}个步骤")
    for i, tc in enumerate(tool_calls):
        name = tc["name"]
        args = tc["args"]
        emit("tool", f"[{i+1}/{len(tool_calls)}] {name}({args})")
        result = call_tool(name, args, emit=emit)
        if str(result).startswith("未知工具") or str(result).startswith("failed"):
            emit("error", str(result))
            return
        #执行失败则停止
    emit("status", f"任务完成 {fmt_robot()}, {fmt_box()}")
#遍历tool_calls执行技能，每步通过emit输出事件
