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
2. 新增 `LLM_Module/visual_rules.py`：
   - 输入：VLM 结构化字典
   - 输出 A：结构化规则结果
   - 输出 B：用于兼容当前 prompt 的稳定摘要文本
3. 更新 `Interactive_Module/interactive.py`：
   - 调用 `describe_structured()`
   - 将结果送入规则层
   - 同时记录原始 VLM 事实和规则层摘要
4. 更新 `LLM_Module/llm_highlevel.py` 的兜底逻辑，不再依赖自由文本关键词匹配，而是依赖规则层结果。

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
- 当证据不足时，规则输出仍然应该保持保守

## 当前状态

- 结构化 VLM JSON 输出：已开始
- 规则层模块：暂未实现
- 规划层接入规则层输出：暂未实现
