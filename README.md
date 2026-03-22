# FinalProject

基于 LLM 决策的四足机器人导航项目，面向 Unitree Go2。

## 当前能力

系统当前只对外暴露 4 个技能：

- `walk`
- `climb`
- `push_box`
- `way_select`

关键约束：

- 最大单步攀爬高度：`0.3 m`
- 技能执行反馈默认等待：`20 秒`
- 执行失败后：直接停止后续任务，不再重规划

## 当前主链路

`user_input -> VLM -> scene_facts -> highlevel plan -> parameter calculation -> lowlevel execution -> tool result`

其中：

- `Interactive_Module/interactive.py`：交互入口
- `VLM_Module/vlm_core.py`：图像转结构化视觉描述
- `LLM_Module/llm_highlevel.py`：高层规划
- `LLM_Module/parameter_calculator.py`：参数计算
- `LLM_Module/llm_lowlevel.py`：低层执行
- `Robot_Module/module/navigation.py`：导航技能实现

## 当前执行后端

`Robot_Module/module/navigation.py` 现在默认不再只是打印 demo，而是直接控制 IsaacLab EnvTest：

- 默认后端：`file`
- 可选后端：`udp`、`ros`
- 默认控制文件：`/tmp/model_use.txt`、`/tmp/envtest_velocity_command.txt`
- 默认控制文件：`/tmp/envtest_goal_command.txt`、`/tmp/envtest_start.txt`、`/tmp/envtest_reset.txt`
- `way_select` 默认按侧向 `walk` 实现；如果希望切到导航策略，可设置：
  - `export FINALPROJECT_WAY_SELECT_POLICY=navigation`
- 如果希望走 UDP server 而不是直接写文件，可设置：
  - `export FINALPROJECT_NAV_BACKEND=udp`

## 输入方式

系统同时使用两类输入：

1. `visual_context`
   - 来自 VLM 的结构化视觉描述
2. `object_facts`
   - 来自 `config/object_facts.json` 的几何真值

`object_facts` 优先级高于 VLM，适合直接提供：

- `navigation_goal`
- `robot_pose`
- `constraints`
- `objects`

示例文件：

- `config/object_facts.example.json`

## 当前规划逻辑

- 单侧可攀爬：`way_select -> climb`
- 箱子辅助攀爬：`way_select -> push_box -> climb -> climb -> walk`
- 如果任务里已经有 `calculated_parameters`，低层直接执行，不再重新猜参数

## 最终运行方式

最短闭环是 4 步：

1. 启动 IsaacLab EnvTest player

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python NewTools/envtest_model_use_player.py --scene_id 4
```

2. 如果你使用 `udp` 后端，再单独启动 UDP server；默认 `file` 后端可跳过这步

```bash
python Socket/envtest_socket_server.py
```

3. 准备几何真值输入

- 默认文件：`/home/xcj/work/FinalProject/config/object_facts.json`
- 至少建议写入：`navigation_goal`、`robot_pose`、`objects`、`constraints`

4. 启动 FinalProject 交互入口

```bash
cd /home/xcj/work/FinalProject/Interactive_Module
python interactive.py
```

然后直接输入类似：

- `前往目标点`
- `到前面的高台上`

系统会自动执行：

- VLM / object_facts 融合
- 高层规划
- 参数计算
- 调用 `walk/climb/push_box/way_select`
- 通过 `Robot_Module/module/navigation.py` 下发到 IsaacLab

如果只想本地看技能编排结果，可直接运行：

```bash
python3 /home/xcj/work/FinalProject/Robot_Module/module/navigation.py case2
python3 /home/xcj/work/FinalProject/Robot_Module/module/navigation.py case4
```

如果只想启动技能服务器给其他上层调用，可运行：

```bash
cd /home/xcj/work/FinalProject/Robot_Module
python skill.py
```

## 相关文档

- `LLM_Module/README.md`
- `Robot_Module/README.md`
- `VLM_Module/prompts/VlmPrompt.yaml`
