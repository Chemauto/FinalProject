from Executor.skills import Nav, Push, climb
from Executor.state import reset_state

def run_demo_task(emit):
    reset_state()
    emit("plan", "目标: Nav(0, 8, 0)\n直线路径被0.5m障碍阻挡，climb最高0.3m，需要箱子辅助")
    emit("tool", "Nav(0, 1, 0)")
    Nav(0, 1, 0, emit=emit)
    emit("tool", "Push(box, 0, 2, 0)")
    Push(0, 2, 0, emit=emit)
    emit("tool", "climb(0.3)")
    climb(0.3, emit=emit)
    emit("tool", "Nav(0, 8, 0)")
    Nav(0, 8, 0, emit=emit)
    emit("status", "任务完成")
#演示执行层如何向TUI发送规划、工具和状态事件
