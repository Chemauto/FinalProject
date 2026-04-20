# Hardware_Module

硬件抽象层。屏蔽 sim/Go2 后端差异，提供统一状态入口。

## 核心接口

```python
from Hardware_Module import get_state

state = get_state()           # 自动检测 task_type（默认 sim）
state = get_state(task_type="go2")  # 指定后端
```

返回标准状态字典，至少包含：
- `connected` — 是否已连接
- `task_type` — 当前后端类型
- `observation` — 观测数据（含 `agent_position`, `environment` 等）
- `runtime` — 运行时状态（含 `snapshot`, `skill`, `model_use`, `goal`, `start` 等）

## 目录结构

```text
Hardware_Module/
  __init__.py       对外暴露 get_state, list_registered_task_types
  registry.py       后端注册表、TaskBackend 加载、sync 接口
  schema.py         统一状态解析引擎
  backends/
    __init__.py
    sim/            IsaacLab 仿真后端
      __init__.py
      schema.py     STATE_SCHEMA + TASK_DATA 定义
      data.py       仿真数据获取（读 /tmp/envtest_*.json）
      state.py      仿真状态辅助函数
    go2/            Unitree Go2 ROS2 后端
      __init__.py
      schema.py     STATE_SCHEMA + TASK_DATA 定义 + ROS2 topic 名字
      data.py       ROS2 订阅节点、数据缓存、get_data() 接口
```

## 各文件职责

### `__init__.py`

包入口，延迟导入 `schema.get_state` 并对外暴露：
- `get_state()` — 获取统一状态
- `list_registered_task_types()` — 列出已注册后端

### `registry.py`

后端注册与管理：

- `TaskBackend` — 后端定义 dataclass（task_type, state_schema, data_getter 等）
- `TaskRegistration` — 注册记录 dataclass
- `register_task()` — 注册新后端
- `load_task_backend()` — 按 task_type 加载后端定义
- `get_task_data()` — 调用后端的 `get_data()` 获取原始数据
- `sync_object_facts_from_live_data()` — 将实时数据同步到 object_facts.json
- `sync_runtime_overrides_from_user_input()` — 根据用户输入覆盖运行时参数

内置注册了两个后端：
- `sim` — IsaacLab 仿真（`Hardware_Module.backends.sim.schema`）
- `go2` — Unitree Go2 ROS2（`Hardware_Module.backends.go2.schema`）

### `schema.py`

统一状态解析引擎：

- `get_state(task_type)` — 主入口，加载后端 → 获取原始数据 → 按 STATE_SCHEMA 解析
- `_parse_schema()` — 递归解析 schema 定义，支持 `source`（从原始数据取值）、`default`（缺省值）、`literal`（固定值）
- `_resolve_path()` — 按 `.` 分隔的路径从嵌套 dict/list 中取值

也提供 CLI 入口：`python -m Hardware_Module.schema --type go2 --json`

### `backends/sim/`

IsaacLab 仿真后端：

- `schema.py` — 定义 `STATE_SCHEMA`（从仿真原始数据到统一状态的映射）和 `TASK_DATA`（后端注册信息）
- `data.py` — 通过读 `/tmp/envtest_*.json` 文件获取仿真数据，包含 `_build_live_snapshot()` 组装状态
- `state.py` — 仿真特定的状态辅助函数

### `backends/go2/`

Unitree Go2 ROS2 后端：

- `schema.py` — 定义 `STATE_SCHEMA`、`TASK_DATA`、`ROS2_TOPICS`（topic 名字定义）
- `data.py` — ROS2 订阅节点，订阅 `/go2/odom`、`/go2/skill_status`、`/go2/scene_objects`，缓存最新数据

## 后端切换

通过环境变量切换：

```bash
export FINALPROJECT_ROBOT_TYPE=sim   # 仿真（默认）
export FINALPROJECT_ROBOT_TYPE=go2   # Go2 真机
```

## 新增后端

1. 在 `backends/` 下创建新目录（如 `backends/new_robot/`）
2. 实现 `schema.py`，定义 `STATE_SCHEMA` 和 `TASK_DATA`
3. 实现 `data.py`，提供 `get_data()` 函数
4. 在 `registry.py` 中调用 `register_task()` 注册

## 与其他模块的关系

- `Excu_Module/state.py` 调用 `Hardware_Module.get_state()` 读取实时状态
- `Data_Module/context.py` 调用 `get_state()` + `sync_object_facts_from_live_data()` 构建上下文
- `Robot_Module/vision/vlm_observe.py` 调用 `get_state()` 获取环境状态
- 不导入项目上层任何模块
