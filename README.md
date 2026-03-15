# FinalProject

## 项目定位

这是一个基于 LLM 决策的四足机器人导航项目，面向 Unitree Go2。
当前代码已经具备以下主链路：

- `Interactive_Module` 接收文字或语音输入
- `VLM_Module` 提供环境文字描述
- `LLM_Module` 负责高层规划和低层工具选择
- `Robot_Module` 提供技能工具
- `Sim_Module` 保留 2D 仿真与动作可视化能力

当前重点是把四足机器人导航技能选择接入这条链路，让机器人在“前往前方目标点”这类任务下，根据环境地形和已有技能，按照“简单、快速”原则选择动作。

## 当前四足导航技能

机器人当前按技能层组织能力，后续可以替换为 IsaacLab 训练出的真实策略。当前对外只保留 4 个技能：

- `walk`
- `climb`
- `push_box`
- `way_select`

约束条件：

- 最大攀爬高度为 `0.3 m`

目前新增了一个导航 demo 模块：

- [navigation_demo.py](/home/xcj/work/FinalProject/Robot_Module/module/navigation_demo.py)

其中 `way_select` 是路线选择技能。机器人初始位于中间位置时，先用这个技能切换到左侧或右侧路线，它内部会调用底层 `walk` 完成横向移动。当前仍是 demo 版本，技能执行先用打印代替，便于后续替换为 IsaacLab / Go2 真正控制接口。

## 已实现的场景 Demo

### 情况 2

环境：

- 左侧有高台，高度 `0.2 m`
- 右侧无障碍

策略：

- 先调用 `way_select` 选择右侧路线
- 再调用 `walk`

当前 demo 会播报：

- 前方左侧检测到高台，高度约 `0.20 m`
- 右侧无障碍
- 按照简单快速原则，选择右侧行走技能

随后调用：

- `way_select`
- `walk`

### 情况 4

环境：

- 左右两侧都有高台，高度均为 `0.4 m`
- 高于最大攀爬高度 `0.3 m`
- 旁边存在一个可推动箱子，高度 `0.2 m`

策略：

- 先调用 `way_select` 选择有箱子的一侧路线
- 再调用 `push_box`
- 再调用 `climb` 爬上箱子
- 再调用 `climb` 从箱子上到高台
- 最后调用 `walk`

当前 demo 会播报：

- 两侧高台高度
- 最大可攀爬高度
- 可推动箱子高度
- 选择“推箱子 + 攀爬 + 行走”组合方案

随后调用：

- `way_select`
- `push_box`
- `climb`
- `climb`
- `walk`

## 与 IsaacLab 的关系

你的四足技能训练参考工程在：

- [Project.md](/home/xcj/work/IsaacLab/IsaacLabBisShe/Project.md)

当前仓库里与 IsaacLab 相关的桥接文件是：

- [walkisaacsim.py](/home/xcj/work/FinalProject/Robot_Module/module/walkisaacsim.py)

本次修改没有直接接入真实策略执行，而是先把“场景判断 -> 路线选择 -> 技能调用”这条链路搭起来。后续你只需要把 `navigation_demo.py` 里 4 个技能的打印逻辑替换为真实策略调用即可。

## 运行方式

### 1. 直接运行本地场景 demo

```bash
python3 Robot_Module/module/navigation_demo.py case2
python3 Robot_Module/module/navigation_demo.py case4
```

### 2. 运行完整交互链路

需要在项目根目录 `.env` 中提供：

- `Test_API_KEY`

启动方式：

```bash
python3 Sim_Module/sim2d/simulator.py
python3 Interactive_Module/interactive.py
python3 Interactive_Module/voice_interactive.py
```

说明：

- `Interactive_Module/interactive.py` 是文本入口
- `Interactive_Module/voice_interactive.py` 是语音入口
- 当前 `Interactive_Module/interactive.py` 默认开启 VLM 上下文
- LLM 提示词已经加入 case 2 / case 4 的导航规则

## 当前代码结构

- [Interactive_Module/interactive.py](/home/xcj/work/FinalProject/Interactive_Module/interactive.py)：文字交互入口
- [Interactive_Module/voice_interactive.py](/home/xcj/work/FinalProject/Interactive_Module/voice_interactive.py)：语音交互入口
- [LLM_Module/llm_core.py](/home/xcj/work/FinalProject/LLM_Module/llm_core.py)：双层 LLM 总入口
- [LLM_Module/prompts/highlevel_prompt.yaml](/home/xcj/work/FinalProject/LLM_Module/prompts/highlevel_prompt.yaml)：高层规划提示词
- [LLM_Module/prompts/lowlevel_prompt.yaml](/home/xcj/work/FinalProject/LLM_Module/prompts/lowlevel_prompt.yaml)：低层执行提示词
- [Robot_Module/skill.py](/home/xcj/work/FinalProject/Robot_Module/skill.py)：机器人技能统一注册入口
- [Robot_Module/module/navigation_demo.py](/home/xcj/work/FinalProject/Robot_Module/module/navigation_demo.py)：四足导航 4 技能模块与 case 2 / case 4 本地演示
- [Robot_Module/module/base.py](/home/xcj/work/FinalProject/Robot_Module/module/base.py)：底盘运动工具
- [Robot_Module/module/chase.py](/home/xcj/work/FinalProject/Robot_Module/module/chase.py)：追击工具
- [Sim_Module/sim2d/simulator.py](/home/xcj/work/FinalProject/Sim_Module/sim2d/simulator.py)：2D 仿真器

## 当前状态

当前版本已经完成：

- 四足导航 4 技能模块注册
- `way_select` 路线选择能力
- case 2 / case 4 的本地技能链演示
- 技能调用打印
- LLM 提示词适配

当前尚未完成：

- 真实 TTS 播报
- 真实地形感知结构化解析
- IsaacLab 训练策略的正式接入
- Go2 实机控制闭环
