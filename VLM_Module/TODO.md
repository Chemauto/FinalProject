# VLM TODO

## 当前方向

当前已经确认的架构方向是：

- `VLM` 只负责报告图像里看到了什么。
- 是否能直接行走、是否需要避障、是否需要处理高度差，由规则层或规划层决定。

这意味着 `VLM` 不应该直接输出下面这类结论：

- “可以安全通过”
- “可以直接走过去”
- “需要 climb”

这些属于下游决策，不属于视觉事实本身。

## 职责划分

### VLM 的职责

`VLM_Module` 只应输出结构化的场景事实，例如：

- 地面或基础平面的描述
- 左侧结构描述
- 右侧结构描述
- 前方区域描述
- 可见障碍物
- 疑似高度差线索
- 明确的不确定项

### 规则层的职责

后续建议新增一个规则模块，暂定为 `LLM_Module/visual_rules.py`，用于消费结构化的 VLM 输出，并产生与运动决策相关的判断。

规则层的候选输出示例：

- `direct_walkable`
- `left_walk_blocked`
- `right_walk_blocked`
- `requires_elevation_action`

需要注意：

- 这里的 `blocked` 指的是“不能只靠纯 walking 通过”，不等于“永远物理不可达”。
- `requires_elevation_action` 表示场景大概率需要处理高度问题，不等于已经证明 `climb` 一定可执行。

## 候选接入路径

推荐后续按下面的方式接入：

1. `VLM_Module.vlm_core.VLMCore.describe_structured()` 返回结构化视觉事实。
2. 新增几何处理层，直接从深度图计算结构化几何结果，而不是让 VLM 去“理解深度图像”。
3. 新增 `LLM_Module/visual_rules.py`：
   - 输入：VLM 结构化字典
   - 输入：深度几何结果
   - 输出 A：结构化规则结果
   - 输出 B：用于兼容当前 prompt 的稳定摘要文本
4. 更新 `Interactive_Module/interactive.py`：
   - 调用 `describe_structured()`
   - 获取深度几何结果
   - 将两者一并送入规则层
   - 同时记录原始 VLM 事实、深度几何结果和规则层摘要
5. 更新 `LLM_Module/llm_highlevel.py` 的兜底逻辑，不再依赖自由文本关键词匹配，而是依赖规则层结果。

## 推荐数据流

建议后续按下面的链路组织：

```text
用户指令
  -> Interactive_Module
  -> 传感器输入层
     -> RGB 图像 -> VLM_Module -> 结构化语义结果
     -> Depth 图像 -> Geometry/Depth 层 -> 结构化几何结果
  -> Rule Layer / visual_rules
  -> LLM Planner
  -> Robot skills
```

可以把它理解成三层职责：

- `RGB + VLM`：回答“这是什么”
- `Depth + Geometry`：回答“多远、多高、是否连续”
- `Rule Layer`：回答“能不能只靠 walk，是否需要处理高度差”

## 深度几何层建议输出

如果后续拿到了深度图，建议不要把深度图直接交给 VLM 做语言描述，而是直接计算几何字段，例如：

- `front_min_distance`
- `left_height_diff`
- `right_height_diff`
- `front_step_height`
- `ground_continuity`
- `has_dropoff`

这些字段的推荐含义：

- `front_min_distance`
  - 机器人正前方最近障碍物距离，单位一般为米。
- `left_height_diff`
  - 左侧路线表面相对当前脚下地面的高度差。
- `right_height_diff`
  - 右侧路线表面相对当前脚下地面的高度差。
- `front_step_height`
  - 正前方若存在台阶、平台边缘或凸起，其高度差估计值。
- `ground_continuity`
  - 前方地面是否连续，`false` 表示可能存在台阶、断层或不连续表面。
- `has_dropoff`
  - 前方是否存在下坠、坑边、悬空等风险。

## 结构化结果示例

### VLM 结构化输出示例

```json
{
  "ground": "黑色网格地面",
  "left_side": "左侧浅绿色平台",
  "right_side": "右侧粉色平台",
  "front_area": "前方存在颜色和平面分界",
  "obstacles": ["左侧平台", "右侧平台"],
  "suspected_height_diff": true,
  "uncertainties": ["无法仅凭RGB确认绝对高度"]
}
```

### 深度几何结果示例

```json
{
  "front_min_distance": 0.72,
  "left_height_diff": 0.18,
  "right_height_diff": 0.31,
  "front_step_height": 0.20,
  "ground_continuity": false,
  "has_dropoff": false
}
```

### 规则层结果示例

```json
{
  "direct_walkable": false,
  "left_walk_blocked": true,
  "right_walk_blocked": true,
  "requires_elevation_action": true,
  "summary": "前方和左右两侧均存在高度差，不能仅靠 walk 通过"
}
```

## 模块落点建议

当前建议的模块拆分如下：

- `VLM_Module/vlm_core.py`
  - 只负责 RGB 语义感知
- `Depth_Module/depth_core.py`
  - 负责读取或接收深度图
- `Depth_Module/geometry_rules.py`
  - 负责从深度图提取距离、高度差、断层等几何结果
- `LLM_Module/visual_rules.py`
  - 负责融合 VLM 语义和深度几何，输出规划可用的规则结果
- `LLM_Module/llm_highlevel.py`
  - 消费规则结果，不直接猜测高度和通行性

## 为什么要这么做

这种职责分离有助于提升：

- 规划输入的稳定性
- 故障定位和解释能力
- 重复测试时的一致性
- 减少 VLM 对通行性过度下结论

## 非目标

这个 TODO 并不意味着只靠 RGB 图像的 VLM 就能变成几何测量器。

即使改成结构化输出，也仍然存在这些限制：

- 单张 RGB 图仍然无法精确给出绝对高度
- 精确可通行性仍然需要规则、几何信息、深度信息或仿真状态
- 深度图也不等于完整世界模型，仍然需要相机标定、视角约束和规则处理
- 当证据不足时，规则输出仍然应该保持保守

## 当前状态

- 结构化 VLM JSON 输出：已开始
- 深度几何层：暂未实现
- 规则层模块：暂未实现
- 规划层接入规则层输出：暂未实现
