# Comm_Module

通信与状态模块，为 LLM agent 提供统一的状态查询和执行反馈接口。

## 设计理念

通用接口只定义**任何 agent 都需要的最抽象概念**，不绑定具体机器人或环境。
具体实现细节全部封装在各自的 Robot 实现中。

```
Status/get_state.py          ← 通用抽象：connected / robot_type / observation
Robot/Sim/                   ← IsaacLab EnvTest 仿真实现
Robot/Go2/                   ← 真机实现（未来）
Robot/Mario/                 ← 超级玛丽实现（示例）
```

## 通用接口 (Status/get_state.py)

```python
from Comm_Module.Status.get_state import get_state

state = get_state()        # 自动检测 robot_type
obs = state["observation"] # agent 能感知到的一切（自由结构）
```

返回结构：

```python
{
    "connected": bool,
    "robot_type": str,
    "observation": {
        # 推荐字段（非强制，每个实现自行决定）
        "agent_position": ...,        agent 在环境中的位置
        "environment": {             环境信息
            "obstacles": [...],
            "terrain": ...,
        },
        "action_result": {           上一步操作结果
            "action": str | None,
            "success": bool | None,
            "feedback": str | None,
        },
        # 实现特定的自由字段...
    }
}
```

`observation` 是完全开放的结构。推荐字段只是为了跨实现的通用 agent 提供参考。

### 命令行

```bash
python -m Comm_Module.Status.get_state              # 默认输出
python -m Comm_Module.Status.get_state --json        # JSON 格式
python -m Comm_Module.Status.get_state --type sim    # 指定类型
python -m Comm_Module.Status.get_state --list-types  # 列出可用类型
```

## 仿真实现 (Robot/Sim/)

### get_state.py — 状态获取

从正在运行的 `envtest_model_use_player.py` 进程和控制文件读取状态，映射为通用 observation：

```python
{
    "agent_position": [x, y, z],          机器人位置
    "environment": {
        "scene_id": 3,                    当前场景
        "obstacles": [                    场景物体
            {"id": "platform_left_low", "type": "platform", "center": [...], "size": [...]}
        ],
        "goal": [5.0, 0.0, 0.0],         导航目标
    },
    "action_result": {
        "skill": "walk",                  当前技能
        "model_use": 1,                   EnvTest 模型编号
        "vel_command": [0.6, 0.0, 0.0],   速度命令
        "start": True,                    是否已启动
    },
    "raw": {                              EnvTest 原始状态（供直接访问）
        "pose_command": ...,
        "status_file_available": True,
    }
}
```

### envtest_status_sync.py — 状态同步

`robot_act` 调用前自动执行 live sync，同步 EnvTest 运行态到 `config/object_facts.json`。

同步来源：运行中的进程、`/tmp/model_use.txt`、`/tmp/envtest_velocity_command.txt`、`/tmp/envtest_goal_command.txt`、`/tmp/envtest_live_status.json`。

## 添加新机器人

创建 `Comm_Module/Robot/<Type>/get_state.py`，实现 `get_state()` 函数：

```python
def get_state() -> dict[str, Any]:
    return {
        "connected": True,
        "robot_type": "<type>",
        "observation": {
            "agent_position": ...,
            "environment": {...},
            "action_result": {...},
        },
    }
```

然后在 `Status/get_state.py` 的 `_ROBOT_BACKENDS` 中注册：

```python
_ROBOT_BACKENDS = {
    "sim": "Comm_Module.Robot.Sim.get_state",
    "mario": "Comm_Module.Robot.Mario.get_state",
}
```

通过环境变量切换：`export FINALPROJECT_ROBOT_TYPE=mario`

## 执行反馈

`execution_comm.py` 提供 ROS2 发布/订阅（`/robot/skill_command`、`/robot/execution_feedback`）。

## 目录结构

```
Comm_Module/
  Status/
    get_state.py                 通用抽象接口
    __init__.py
  Robot/
    Sim/
      get_state.py               仿真状态获取
      envtest_status_sync.py     EnvTest 状态同步
      __init__.py
    __init__.py
  Backup/                        原始备份
  execution_comm.py              ROS2 执行反馈通信
  __init__.py
```
