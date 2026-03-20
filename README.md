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

## 常用入口

交互运行：

```bash
cd /home/robot/work/FinalProject/Interactive_Module
python interactive.py
```

导航模块本地 demo：

```bash
python3 Robot_Module/module/navigation.py case2
python3 Robot_Module/module/navigation.py case4
```

## 相关文档

- `LLM_Module/README.md`
- `Robot_Module/README.md`
- `VLM_Module/prompts/VlmPrompt.yaml`
