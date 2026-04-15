# Robot_Module

## 作用

`Robot_Module` 是机器人能力注册层。
它把底层视觉感知模块、动作执行模块和上层 agent 工具统一注册成可调用技能。

当前主入口是 [skill.py](skill.py)。

## 当前结构

- [agent_tools.py](agent_tools.py)
  上层 agent 工具，提供 `robot_act`；内部调用 `LLM_Module` 的规划/参数计算/执行链
  使用 `_StreamingBuffer` + `redirect_stdout` 捕获内部 pipeline stdout，通过 `log_callback` 转发
- [module/Vision/vlm.py](module/Vision/vlm.py)
  视觉感知模块，提供 `vlm_observe` 和 `register_tools(mcp)`
- [module/Action/navigation.py](module/Action/navigation.py)
  动作执行模块，提供 7 个导航/攀爬/推箱技能；内部日志输出到 stderr
- [module/Action/example.py](module/Action/example.py)
  示例模块

## 对外暴露方式

最外层智能体视角下，主要使用 2 个高层工具：

- `vlm_observe`（来自 `module/Vision/vlm.py`）
- `robot_act`（来自 `agent_tools.py`）

`robot_act` 内部继续复用动作技能链。

## 内部动作技能

当前 `Action/navigation.py` 提供 7 个动作技能：

- `walk`：沿当前路线前进
- `navigation`：地面自动导航到目标点
- `nav_climb`：直接翻越高台前往目标点
- `climb_align`：攀爬前对正
- `climb`：单步攀爬，默认上限 `0.3 m`
- `push_box`：推动箱子到目标位置或自动目标
- `way_select`：从中间切换到左侧或右侧路线

## 日志输出

- `navigation.py` 内部通过 `print(..., file=sys.stderr)` 输出 `[go2.skill]`、`[go2.speech]` 等日志
- 交互模式下，`interactive.py` 使用 `redirect_stderr` 抑制这些噪音
- `agent_tools.py` 的 `_StreamingBuffer` 捕获 `llm_core.py` / `llm_lowlevel.py` 的 stdout 输出，通过 `log_callback` 转发

## 注册关系

[skill.py](skill.py) 会统一注册：

- `module/Action/navigation.py`
- `module/Vision/vlm.py`
- `agent_tools.py`

## 当前执行方式

- 通过 FastMCP 注册工具
- 技能执行结果统一封装为结构化反馈
- `module/Action/navigation.py` 默认直接写 IsaacLab EnvTest 控制文件
- 可选执行后端：`file` / `udp` / `ros`
- `way_select` 默认按侧向 `walk` 控制；如需切到导航策略可设置：
  - `export FINALPROJECT_WAY_SELECT_POLICY=navigation`
- `push_box.target_position` 支持：`"auto"` / `"x,y,z"` / `"[x, y, z]"`

## 本地运行

直接启动交互入口：

```bash
cd /home/robot/work/FinalProject/Interactive_Module
python interactive.py
```

如果你想单独启动技能服务器：

```bash
cd /home/robot/work/FinalProject/Robot_Module
python skill.py
```

如果你想单独跑动作模块 demo：

```bash
python3 /home/robot/work/FinalProject/Robot_Module/module/Action/navigation.py case2
python3 /home/robot/work/FinalProject/Robot_Module/module/Action/navigation.py case4
```

## 与上层的关系

通常由 [Interactive_Module/interactive.py](../Interactive_Module/interactive.py) 间接调用：

- 顶层智能体决定直接回复、调用 `vlm_observe`，或调用 `robot_act`
- `robot_act` 内部复用 `LLM_Module` 的规划、参数计算和低层执行
- `Robot_Module` 负责真正调用视觉或动作技能
