# Comm_Module

通信与状态模块，目标是把“统一状态入口”和“具体数据来源”彻底解耦。

## 当前抽象

`Comm_Module` 现在分成 3 层：

1. `Status`
   只负责统一入口和通用解析。
2. `Task/<Type>/Data.py`
   只负责定义该后端的状态格式与元信息。
3. `Task/<Type>/get_data.py`
   只负责获取该后端的真实原始数据。

也就是说：

- `Data.py` 定义“状态应该长什么样”
- `get_data.py` 定义“原始数据从哪里来”
- `Status/get_state.py` 负责“按定义自动解析成统一状态”

## 当前结构

```text
Comm_Module/
  Status/
    get_state.py                 通用状态入口 + 通用 schema 解析器
  Task/
    __init__.py                  task 注册表与统一分发
    Sim/
      Data.py                    sim 后端元信息 + 状态格式定义
      get_data.py                获取 EnvTest 原始数据 + 同步 object_facts
      __init__.py
  execution_comm.py              可选执行反馈通信
```

## 各层职责

### `Status/get_state.py`

对外统一只暴露一个入口：

```python
from Comm_Module import get_state

state = get_state()
```

它内部做 3 件事：

1. 根据 `task_type` 找到已注册 backend
2. 调用该 backend 的 `get_data.py` 获取 raw data
3. 按 `Data.py` 里定义的 `state_schema` 自动解析出统一状态

`Status` 不再依赖每个 backend 手写 `get_state()`。

当前 `get_state()` 的返回值至少包含：

- `connected`
- `task_type`
- `observation`
- `runtime`

### `Task/__init__.py`

这是 backend 注册中心。

当前负责：

- 注册可用 backend
- 加载 backend 定义
- 调用 backend 对应的 `get_data`
- 统一分发 `sync_object_facts_from_live_data()` 等同步接口

当前默认注册：

```python
register_task(
    task_type="sim",
    module_path="Comm_Module.Task.Sim.Data",
    description="IsaacLab EnvTest 仿真后端",
)
```

### `Task/Sim/Data.py`

这个文件现在只定义：

- `sim` backend 的元信息
- `state_schema`

例如：

- `observation.agent_position <- snapshot.robot_pose`
- `observation.environment.goal <- snapshot.goal`
- `observation.raw.process <- process`
- `runtime.snapshot <- snapshot`
- `runtime.scene_objects <- scene_objects`

它不直接获取数据，也不自己拼状态。

### `Task/Sim/get_data.py`

这个文件现在只负责真实原始数据：

- 找 `envtest_model_use_player.py`
- 读 `/tmp` 控制文件
- 读 live status JSON
- 构建场景物体列表
- 同步 `config/object_facts.json`

它返回的是 raw payload，不负责通用状态映射。

## 当前标准状态

当前 `sim` 后端解析出的统一状态，执行层主要会使用这些字段：

```python
state = {
    "connected": bool,
    "task_type": "sim",
    "observation": {
        "agent_position": [...],
        "environment": {
            "scene_id": ...,
            "obstacles": [...],
            "goal": ...,
        },
    },
    "runtime": {
        "timestamp": ...,
        "snapshot": {...},
        "skill": ...,
        "model_use": ...,
        "goal": ...,
        "start": ...,
        "scene_objects": [...],
    },
}
```

`Excu_Module` 只依赖这份标准状态，不再自己直接解析具体状态文件。

## 当前 Sim 后端的数据流

```text
上层 agent / Excu_Module
-> Comm_Module.get_state()
-> Comm_Module.Task 加载 sim backend 定义
-> Comm_Module.Task.Sim.get_data.get_data()
-> 返回 raw data
-> Status/get_state.py 按 Data.py.state_schema 自动解析
-> 返回统一状态
```

同步 `object_facts.json` 的路径：

```text
robot_act
-> Comm_Module.Task.sync_object_facts_from_live_data()
-> Comm_Module.Task.Sim.get_data.sync_object_facts_from_live_data()
-> 更新 config/object_facts.json
```

## 对上层的推荐用法

### 获取统一状态

```python
from Comm_Module import get_state

state = get_state()
```

### 获取某个 backend 的原始数据

```python
from Comm_Module.Task import get_task_data

raw_data = get_task_data("sim")
```

### 手动同步 live data 到 object_facts

```python
from Comm_Module.Task import sync_object_facts_from_live_data

payload = sync_object_facts_from_live_data("/path/to/object_facts.json", user_input="前往20,0,0")
```

## 命令行

### 通用状态入口

```bash
python -m Comm_Module.Status.get_state
python -m Comm_Module.Status.get_state --json
python -m Comm_Module.Status.get_state --type sim
python -m Comm_Module.Status.get_state --list-types
```

### 查看 Sim 状态格式定义

```bash
python -m Comm_Module.Task.Sim.Data
```

### 查看 Sim 原始数据

```bash
python -m Comm_Module.Task.Sim.get_data
python -m Comm_Module.Task.Sim.get_data --json
```

### 手动同步 Sim 状态

```bash
python -m Comm_Module.Task.Sim.get_data --live-envtest
python -m Comm_Module.Task.Sim.get_data --status-file /path/to/status.txt
python -m Comm_Module.Task.Sim.get_data --live-envtest --user-input "前往20,0,0"
```

## 添加新后端

如果以后增加真机或其他环境，建议保持同样模式：

```text
Comm_Module/Task/<Type>/
  Data.py
  get_data.py
```

其中：

- `Data.py` 只定义 backend 元信息和 `state_schema`
- `get_data.py` 只负责真实取数
- `Status/get_state.py` 自动完成统一状态解析

然后在 `Comm_Module/Task/__init__.py` 注册：

```python
register_task(
    task_type="go2",
    module_path="Comm_Module.Task.Go2.Data",
    description="Unitree Go2 真机后端",
)
```

通过环境变量切换默认 backend：

```bash
export FINALPROJECT_ROBOT_TYPE=go2
```

## 执行反馈

`execution_comm.py` 负责执行反馈通信，目前保留 ROS2 话题：

- `/robot/skill_command`
- `/robot/execution_feedback`

## 一句话总结

`Status` 负责统一解析，`Data.py` 负责定义格式，`get_data.py` 负责提供原始数据。`Excu_Module` 只通过这里拿统一状态。
