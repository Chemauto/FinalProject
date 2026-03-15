# CLAUDE.md

本文件面向在本仓库内工作的 Claude Code / Codex 类代理，描述当前代码结构、四足导航目标、已接入的 demo 模块和推荐工作方式。

## 项目目标

本项目正在从原有的“双层 LLM + 2D 仿真工具调用”框架，逐步扩展为“基于 LLM 思考的四足机器人导航系统”。

目标任务是：

- 用户通过文字或语音下达指令，例如“前往前方目标点”
- 系统结合视觉环境信息判断当前地形
- LLM 在已有技能中做选择
- 机器人先给出简短可解释的语音说明
- 再调用合适的技能完成导航

核心决策原则：

- 简单
- 快速

当前重点技能：

- `walk`
- `climb`
- `push_box`
- `way_select`

约束：

- 最大攀爬高度为 `0.3 m`

## 当前已落地的导航 Demo

已新增文件：

- [navigation_demo.py](/home/xcj/work/FinalProject/Robot_Module/module/navigation_demo.py)

该模块已注册进：

- [skill.py](/home/xcj/work/FinalProject/Robot_Module/skill.py)

当前覆盖两个重点场景：

### Case 2

- 左侧存在 `0.2 m` 高台
- 右侧无障碍
- 决策：先 `way_select` 到右侧，再 `walk`

当前行为：

- 打印机器人语音播报
- 调用 `way_select`
- 调用 `walk`
- 返回结构化 JSON 结果

### Case 4

- 左右两侧存在 `0.4 m` 高台
- 超过最大攀爬高度 `0.3 m`
- 附近存在 `0.2 m` 可推动箱子
- 决策：先 `way_select` 到有箱子的一侧，再 `push_box`，再两次 `climb`，最后 `walk`

当前行为：

- 打印机器人语音播报
- 调用 `way_select`
- 调用 `push_box`
- 调用两次 `climb`
- 调用 `walk`
- 返回结构化 JSON 结果

注意：

- 当前“说话”仍是 `stderr` 打印，不是真实 TTS
- 当前“技能执行”仍是 demo 打印，不是真实 IsaacLab / Go2 控制

## 代码结构

### 1. `Interactive_Module`

交互入口与整体调度。

- [interactive.py](/home/xcj/work/FinalProject/Interactive_Module/interactive.py)
  - 文本交互入口
  - 初始化工具注册
  - 初始化双层 LLM
  - 默认开启 VLM 上下文
  - 调用 `LLM_Module` 完成任务规划与执行

- [voice_interactive.py](/home/xcj/work/FinalProject/Interactive_Module/voice_interactive.py)
  - 语音交互入口
  - 调用 ASR 后复用 `interactive.py` 的处理流程

- [asr_core.py](/home/xcj/work/FinalProject/Interactive_Module/asr_core.py)
  - 负责录音与语音转文字

### 2. `LLM_Module`

双层 LLM 决策核心。

- [llm_core.py](/home/xcj/work/FinalProject/LLM_Module/llm_core.py)
  - 高层规划 + 低层执行总入口

- [llm_highlevel.py](/home/xcj/work/FinalProject/LLM_Module/llm_highlevel.py)
  - 把用户目标拆成子任务
  - 目前已补充“前往目标点”导航任务的规划规则

- [llm_lowlevel.py](/home/xcj/work/FinalProject/LLM_Module/llm_lowlevel.py)
  - 根据单个子任务选择工具
  - 目前已补充 case 2 / case 4 对应工具选择规则

- [highlevel_prompt.yaml](/home/xcj/work/FinalProject/LLM_Module/prompts/highlevel_prompt.yaml)
  - 高层提示词

- [lowlevel_prompt.yaml](/home/xcj/work/FinalProject/LLM_Module/prompts/lowlevel_prompt.yaml)
  - 低层提示词

### 3. `Robot_Module`

技能层与动作层。

- [skill.py](/home/xcj/work/FinalProject/Robot_Module/skill.py)
  - 统一注册机器人技能
  - 当前已注册 `base`、`chase`、`navigation_demo`

- [base.py](/home/xcj/work/FinalProject/Robot_Module/module/base.py)
  - 基础底盘运动工具

- [chase.py](/home/xcj/work/FinalProject/Robot_Module/module/chase.py)
  - 原有追击相关工具

- [navigation_demo.py](/home/xcj/work/FinalProject/Robot_Module/module/navigation_demo.py)
  - 四足导航技能 demo
  - 包含：
    - `walk`
    - `climb`
    - `push_box`
    - `way_select`

- [walkisaacsim.py](/home/xcj/work/FinalProject/Robot_Module/module/walkisaacsim.py)
  - IsaacSim / IsaacLab 的运动控制桥接原型
  - 当前未直接接入新的导航 demo 流程

### 4. `VLM_Module`

环境理解入口。

- [vlm_core.py](/home/xcj/work/FinalProject/VLM_Module/vlm_core.py)
  - 获取图像并输出中文环境描述

- [image_source.py](/home/xcj/work/FinalProject/VLM_Module/image_source.py)
  - 摄像头图像输入

### 5. `Sim_Module`

当前仍保留 2D 仿真环境。

- [simulator.py](/home/xcj/work/FinalProject/Sim_Module/sim2d/simulator.py)
  - 接收动作命令并显示可视化结果

## 当前主数据流

### 文字输入

```text
用户文字指令
  ↓
Interactive_Module/interactive.py
  ↓
VLM_Module/vlm_core.py（默认开启）
  ↓
LLM_Module/llm_highlevel.py
  ↓
LLM_Module/llm_lowlevel.py
  ↓
Robot_Module/skill.py
  ↓
Robot_Module/module/navigation_demo.py 或其他工具
```

### 语音输入

```text
麦克风语音
  ↓
Interactive_Module/asr_core.py
  ↓
Interactive_Module/voice_interactive.py
  ↓
复用 interactive.py 同一套流程
```

## 当前建议的导航理解方式

对于“前往前方目标点”这类任务，优先让高层把任务拆成多个技能步骤，再让低层 LLM 逐步调用 `way_select`、`walk`、`climb`、`push_box`。

推荐映射：

- 若视觉信息说明“左侧高台约 0.2 m，右侧无障碍”，先调用 `way_select(direction="right")`，再调用 `walk`
- 若视觉信息说明“左右高台约 0.4 m，存在约 0.2 m 可推动箱子”，先调用 `way_select`，再调用 `push_box`、`climb`、`climb`、`walk`

这样后续替换成真实技能控制时，接口更稳定。

## 与 IsaacLab 的关系

四足技能训练参考工程在：

- [Project.md](/home/xcj/work/IsaacLab/IsaacLabBisShe/Project.md)

当前推荐做法：

1. 保持 `navigation_demo.py` 的 4 技能接口不变
2. 先把 `walk` / `climb` / `push_box` / `way_select` 作为统一技能调用层
3. 后续把 demo 打印替换为 IsaacLab 训练策略调用
4. 最后再接入真实 Go2 的执行链路和 TTS

## 开发时的注意点

- 当前项目仍混合了旧的 2D 追击逻辑和新的四足导航 demo，不要误删现有追击模块
- 新的四足技能优先写在 `Robot_Module/module` 下
- 如果只做本地演示，不必强依赖 OpenAI API，可以直接运行：

```bash
python3 Robot_Module/module/navigation_demo.py case2
python3 Robot_Module/module/navigation_demo.py case4
```

- 如果要跑完整 LLM 链路，需要根目录 `.env` 中配置 `Test_API_KEY`

## 当前缺口

下面这些能力还没有真正接上：

- 结构化环境感知输出
- 真正的 TTS 播报
- IsaacLab 技能策略到导航工具的正式绑定
- Go2 实机控制
- case 1 / case 3 的正式工具化

如果后续继续扩展，优先方向应是：

1. 把 VLM 输出转成结构化地形参数
2. 扩展 `navigation_demo.py` 覆盖 case 1 / case 3
3. 把 demo 技能替换为真实策略执行
