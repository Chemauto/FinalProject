from Executor.tools import call_tool
from Executor.state import format_latest_state

ACTION_TOOLS = {"nav", "nav_climb", "walk_skill", "push", "climb"}

def run_plan(tool_calls, emit):
    results = []
    for i, tc in enumerate(tool_calls):
        name = tc["name"]
        args = tc["args"]
        emit("tool", f"[{i+1}/{len(tool_calls)}] {name}({args})")
        result = call_tool(name, args, emit=emit)
        results.append(_normalize_result(name, args, result))
        if results[-1]["signal"] == "FAILURE":
            emit("error", results[-1]["message"])
            return results
        #执行失败则停止
    if any(tc["name"] in ACTION_TOOLS for tc in tool_calls):
        emit("status", f"动作完成 {format_latest_state()}")
    return results
#遍历tool_calls执行技能，每步通过emit输出事件

def _normalize_result(name, args, result):
    if isinstance(result, dict):
        signal = result.get("signal")
        if signal is None:
            signal = "SUCCESS" if result.get("status") == "success" else "FAILURE"
        return {
            "name": name,
            "args": args,
            "signal": signal,
            "message": result.get("message") or result.get("summary") or str(result),
            "result": result,
        }
    signal = "FAILURE" if str(result).startswith("未知工具") or str(result).startswith("failed") else "SUCCESS"
    return {"name": name, "args": args, "signal": signal, "message": str(result), "result": result}
#把工具返回值统一成Executor结果结构
