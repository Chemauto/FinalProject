# Robot_Module

## 作用

`Robot_Module` 是机器人能力注册层。
它把底层视觉感知模块、动作执行模块和上层 agent 工具统一注册成可调用技能。

当前主入口是 [skill.py](/home/robot/work/FinalProject/Robot_Module/skill.py)。

## 当前结构

- [agent_tools.py](/home/robot/work/FinalProject/Robot_Module/agent_tools.py)
  上层 agent 工具，只保留 `robot_act`
- [module/Vision/vlm.py](/home/robot/work/FinalProject/Robot_Module/module/Vision/vlm.py)
  视觉感知模块，提供 `vlm_observe` 和 `register_tools(mcp)`
- [module/Action/navigation.py](/home/robot/work/FinalProject/Robot_Module/module/Action/navigation.py)
  动作执行模块，提供 7 个导航/攀爬/推箱技能
- [module/Action/example.py](/home/robot/work/FinalProject/Robot_Module/module/Action/example.py)
  示例模块

## 对外暴露方式

最外层智能体视角下，主要使用 2 个高层工具：

- `vlm_observe`
- `robot_act`

其中：

- `vlm_observe` 来自 `module/Vision/vlm.py`
- `robot_act` 来自 `agent_tools.py`

`robot_act` 内部继续复用动作技能链。

## 内部动作技能

当前 `Action/navigation.py` 提供 7 个动作技能：

- `walk`
- `navigation`
- `nav_climb`
- `climb_align`
- `climb`
- `push_box`
- `way_select`

说明：

- `walk`：沿当前路线前进
- `navigation`：地面自动导航到目标点
- `nav_climb`：直接翻越高台前往目标点
- `climb_align`：攀爬前对正
- `climb`：单步攀爬，默认上限 `0.3 m`
- `push_box`：推动箱子到目标位置或自动目标
- `way_select`：从中间切换到左侧或右侧路线

## 注册关系

[skill.py](/home/robot/work/FinalProject/Robot_Module/skill.py) 会统一注册：

- `module/Action/navigation.py`
- `module/Vision/vlm.py`
- `agent_tools.py`

也就是：

- 感知模块自己注册感知技能
- 动作模块自己注册动作技能
- 上层 agent 模块只注册高层封装技能

## 当前执行方式

- 通过 FastMCP 注册工具
- 技能执行结果统一封装为结构化反馈
- `module/Action/navigation.py` 默认直接写 IsaacLab EnvTest 控制文件
- 可选执行后端：
  - `file`
  - `udp`
  - `ros`
- `way_select` 默认按侧向 `walk` 控制；如需切到导航策略可设置：
  - `export FINALPROJECT_WAY_SELECT_POLICY=navigation`
- `push_box.target_position` 支持：
  - `"auto"`
  - `"x,y,z"`
  - `"[x, y, z]"`

## 本地运行

直接启动交互入口：

```bash
cd /home/xcj/work/FinalProject/Interactive_Module
python interactive.py
```

如果你想单独启动技能服务器：

```bash
cd /home/xcj/work/FinalProject/Robot_Module
python skill.py
```

如果你想单独跑动作模块 demo：

```bash
python3 /home/xcj/work/FinalProject/Robot_Module/module/Action/navigation.py case2
python3 /home/xcj/work/FinalProject/Robot_Module/module/Action/navigation.py case4
```

## 与上层的关系

通常由 [Interactive_Module/interactive.py](/home/robot/work/FinalProject/Interactive_Module/interactive.py) 间接调用：

- 顶层智能体决定直接回复、调用 `vlm_observe`，或调用 `robot_act`
- `robot_act` 内部复用 `LLM_Module` 的规划、参数计算和低层执行
- `Robot_Module` 负责真正调用视觉或动作技能
