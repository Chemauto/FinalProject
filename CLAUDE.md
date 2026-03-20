# FinalProject

这是一个基于 LLM 决策的四足机器人导航项目，面向 Unitree Go2。  
当前系统已经打通这条链路：

`用户指令 -> VLM 感知 -> scene_facts 融合 -> 高层规划 -> parameter calculation -> 低层执行 -> 技能反馈`

当前对外只保留 4 个技能：

- `walk`
- `climb`
- `push_box`
- `way_select`

其中最大单步攀爬高度为 `0.3 m`。

## 当前流程

交互入口在 [interactive.py](/home/robot/work/FinalProject/Interactive_Module/interactive.py)。

一次完整运行的流程是：

1. 用户输入指令，例如“前往目标点”。
2. 系统先尝试读取结构化几何真值 `config/object_facts.json`。
3. 如果启用 VLM，则调用一次 VLM，得到结构化视觉描述。
4. VLM 输出被转成 `scene_facts`。
5. 如果存在 `object_facts`，系统会把它和 `scene_facts` 融合。
   `object_facts` 优先，VLM 只补语义描述和不确定项。
6. 高层规划器根据：
   - `user_input`
   - `visual_context`
   - `scene_facts`
   - `object_facts`
   生成候选方案和最终任务链。
7. 在执行前，`parameter_calculator` 会把抽象任务补成具体参数。
8. 低层执行器优先使用已经算好的参数直接调用技能；没有几何参数时才让低层 LLM 补参数。
9. 如果某一步失败，总控会直接停止后续任务，并输出失败反馈。
10. 技能执行反馈默认等待超时为 `20 秒`。

核心实现分别在：

- [interactive.py](/home/robot/work/FinalProject/Interactive_Module/interactive.py)
- [vlm_core.py](/home/robot/work/FinalProject/VLM_Module/vlm_core.py)
- [llm_highlevel.py](/home/robot/work/FinalProject/LLM_Module/llm_highlevel.py)
- [parameter_calculator.py](/home/robot/work/FinalProject/LLM_Module/parameter_calculator.py)
- [llm_lowlevel.py](/home/robot/work/FinalProject/LLM_Module/llm_lowlevel.py)
- [llm_core.py](/home/robot/work/FinalProject/LLM_Module/llm_core.py)

## 数据输入

### 1. VLM 输入

VLM 当前负责图像语义理解，输出结构化视觉描述，例如：

- 左侧/右侧是什么地形
- 是否存在疑似高度差
- 是否有障碍物
- 哪些地方仍不确定

这部分由 [vlm_core.py](/home/robot/work/FinalProject/VLM_Module/vlm_core.py) 生成，并被转换成导航规划更容易使用的 `scene_facts`。

### 2. 结构化几何真值输入

你现在希望提供的“箱子坐标、尺寸、高台坐标、尺寸、机器人位姿、目标点”这类信息，不再让 VLM 猜，而是直接通过 JSON 文件输入。

默认路径是：

- `config/object_facts.json`

仓库里提供了示例：

- [object_facts.example.json](/home/robot/work/FinalProject/config/object_facts.example.json)

当前支持的 JSON 结构是：

```json
{
  "navigation_goal": [5.0, 0.0, 0.0],
  "robot_pose": [0.0, 0.0, 0.0],
  "constraints": {
    "max_climb_height_m": 0.3,
    "push_only_on_ground": true,
    "climb_requires_adjacency": true
  },
  "objects": [
    {
      "id": "box1",
      "type": "box",
      "center": [2.3, 0.25, 0.0],
      "size": [0.8, 0.5, 0.2],
      "movable": true
    },
    {
      "id": "platform_front",
      "type": "platform",
      "center": [3.5, 0.12, 0.0],
      "size": [0.6, 0.6, 0.4],
      "movable": false
    }
  ]
}
```

字段含义：

- `navigation_goal`: 目标点世界坐标 `[x, y, z]`
- `robot_pose`: 当前机器人位姿，这里先按位置 `[x, y, z]` 使用
- `constraints`: 当前规划约束
- `objects`: 场景中的结构化物体列表
- `objects[i].type`: 当前主要支持 `box` 和 `platform`
- `objects[i].center`: 物体中心坐标 `[x, y, z]`
- `objects[i].size`: 物体尺寸 `[length, width, height]`
- `objects[i].movable`: 是否可推动

[object_facts_loader.py](/home/robot/work/FinalProject/LLM_Module/object_facts_loader.py) 会在读入时做归一化：

- 检查 `navigation_goal` 和 `robot_pose` 必须是长度为 3 的数组
- 检查每个对象的 `center` 和 `size` 必须是长度为 3 的数组
- 把数字统一转成 `float`
- 给约束补默认值

## scene_facts 融合规则

融合逻辑在 [vlm_core.py](/home/robot/work/FinalProject/VLM_Module/vlm_core.py)。

规则很简单：

- 没有 `object_facts` 时，只使用 VLM 生成的 `scene_facts`
- 有 `object_facts` 时，先从几何真值生成一份结构化 `scene_facts`
- 再把 VLM 的 `uncertainties` 和视觉摘要补进去
- 如果两者冲突，以 `object_facts` 为准

这意味着现在的系统是：

- VLM 负责“看见了什么、哪里不确定”
- `object_facts` 负责“几何真值是什么”
- 高层规划优先按几何真值做决策

## 高层规划怎么做决策

高层规划在 [llm_highlevel.py](/home/robot/work/FinalProject/LLM_Module/llm_highlevel.py)。

当前不是纯 prompt 决策，还加了一层规则修正，主要解决两类你关心的场景：

1. 单侧可攀爬
   - 例如左侧 `0.2 m`，右侧 `0.4 m`，没有箱子
   - 系统会把错误的 `walk` 修正成：
   - `way_select -> climb`

2. 箱子辅助攀爬
   - 例如箱子 `0.2 m`，高台 `0.4 m`
   - 只要满足：
   - `box_height <= 0.3`
   - `platform_height > 0.3`
   - `platform_height - box_height <= 0.3`
   - 系统就会把错误的 `walk` 修正成：
   - `way_select -> push_box -> climb -> climb -> walk`

这一步只决定技能顺序和对象关系，不在高层里直接输出精确坐标。

## Parameter Calculation 怎么实现

这部分在 [parameter_calculator.py](/home/robot/work/FinalProject/LLM_Module/parameter_calculator.py)。

它的作用是把高层的抽象任务补成具体参数。

当前已经实现的规则包括：

1. `way_select`
   - 通过目标平台或支撑箱子的 `center[1]` 判断左右侧
   - `y >= 0` 视为 `left`
   - `y < 0` 视为 `right`

2. `push_box`
   - 选择最合适的 `box` 作为支撑物
   - 选择最高的 `platform` 作为目标高台
   - 生成贴近高台边缘的推箱子目标点

当前使用的目标点计算是：

```text
x = platform_center.x - platform_size.length / 2 - box_size.length / 2
y = platform_center.y
z = box_center.z
```

3. `climb`
   - 第一段攀爬到箱子顶部时：`height = box_height`
   - 第二段从箱子到高台时：`height = platform_height - box_height`
   - 单段攀爬场景时：`height = platform_height`

也就是说，当前系统已经把论文里那种：

- 当前计划是什么
- 机器人位姿是什么
- 物体描述是什么
- 根据对象相对关系计算目标位置

这一步拆成了一个独立模块，而不是继续让低层 LLM 纯靠文字猜。

## 低层执行怎么用这些参数

低层执行在 [llm_lowlevel.py](/home/robot/work/FinalProject/LLM_Module/llm_lowlevel.py)。

现在的行为是：

- 如果任务里已经带了 `calculated_parameters`
  - 直接按这些参数调用工具
  - 不再让低层 LLM 重新猜 `height`、`target_position`、`direction`
- 如果没有 `calculated_parameters`
  - 仍走原来的低层 LLM 工具调用逻辑

这能明显减少这类错误：

- 明明应该 `climb(0.2)`，却被模型写成 `climb(0.4)`
- 明明应该推到高台边缘，却把 `push_box.target_position` 写成泛化文本

## 失败处理与超时

总控在 [llm_core.py](/home/robot/work/FinalProject/LLM_Module/llm_core.py)。

当前策略是：

- 正常情况下按高层给出的任务链顺序执行
- 如果某一步失败，立即停止后续任务，不再重新规划
- 技能执行反馈默认等待 `20 秒`
- 超时后会返回统一失败反馈，消息中会包含超时时长

## 你要怎么传数据

如果你是直接跑交互入口：

```bash
cd /home/robot/work/FinalProject/Interactive_Module
python interactive.py
```

那默认读取的是：

- `config/object_facts.json`

也就是说，你的数据模块现在最简单的接法就是：

1. 上游模块产出一份 JSON
2. 把它写到 `config/object_facts.json`
3. 交互入口自动读取
4. 高层规划和参数计算自动优先使用这份几何真值

如果你不是从 CLI 跑，而是从 Python 里直接调用，也可以给 [process_user_input](/home/robot/work/FinalProject/Interactive_Module/interactive.py) 传 `object_facts_path`，指定其他 JSON 路径。

## 当前限制

当前实现已经能稳定覆盖这些模式：

- 单侧可攀爬
- 箱子辅助攀爬
- VLM + 几何真值融合
- 失败即停止

但它仍然是一个轻量 demo，当前限制包括：

- `robot_pose` 现在主要只用到了位置，没有完整姿态推导
- `parameter_calculator` 目前主要支持 `box` 和 `platform`
- `push_box` 的目标点还是简单几何规则，不是完整物理规划
- “物体邻接”约束目前更多是通过几何规则隐式满足，还没有完整接触/拓扑判断

## 建议的上游数据模块输出

如果你后面要把 perception/map 模块正式接进来，建议直接输出这种结构化信息：

- `navigation_goal`
- `robot_pose`
- `constraints`
- `objects`

而不是把这些再包装成自然语言交给 VLM。

原因很直接：

- 视觉语义适合交给 VLM
- 几何真值不适合交给 VLM猜
- 几何数据应该直接进 planner 和 parameter calculation

这也是当前这版实现的核心思路。
