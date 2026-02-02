# LLM_Module 待完成功能清单

本文档列出了需要实现的功能，按优先级和依赖关系排序。

## 📋 总览

- [x] 第一阶段：VLM 环境理解集成 ✅ 已完成
- [ ] 第二阶段：执行监控逻辑实现（2-3小时）
- [ ] 第三阶段：后台监控启用（1小时）
- [ ] 第四阶段：测试和验证（1小时）

---

## 🔵 第一阶段：VLM 环境理解集成 ✅ 已完成

**目标**：让 High-Level LLM 能够理解视觉环境信息

**完成时间**：2025年2月2日

**架构说明**：
- **High-Level LLM** 支持两种 VLM 模式：
  1. **本地 Ollama**（默认）：使用 `qwen3-vl:4b` 进行环境理解
  2. **远程 API**：使用 `qwen-vl-plus` 等云端模型
- **Low-Level LLM** 通过 `detect_color_and_act` 工具调用 **VLM_Module** 进行**颜色检测**

**测试结果**：
- ✅ 本地 Ollama 模式测试通过
- ✅ 环境理解输出正确（识别红色方块、描述环境）

### ✅ 任务 1.1：创建 VLM 环境理解提示词

**状态**：✅ 完成

**文件**：`LLM_Module/prompts/vlm_perception.yaml`（已创建）

**完成内容**：
- ✅ 创建了 VLM 提示词文件
- ✅ 定义了 VLM 的角色和职责
- ✅ 包含环境观察要求（空间、目标、颜色、建议）
- ✅ 提供了清晰的分析指导

---

### ✅ 任务 1.2：修改 `high_level_llm.py`

**状态**：✅ 完成（支持本地 Ollama + 远程 API）

**重要修正（2025年2月2日）**：
- ❌ **错误方式**：尝试调用 `VLM_Module.vlm_core_remote.VLMClient`（不兼容）
- ✅ **正确方式**：支持两种 VLM 模式
  1. **本地 Ollama**（默认）：使用 `ollama.run qwen3-vl:4b`
  2. **远程 API**：使用 OpenAI 兼容 API（如 `qwen-vl-plus`）
- 📝 **原因**：VLM_Module 是专用的**颜色检测器**，High-Level LLM 需要**通用环境理解**

**修改内容**：
- ✅ `__init__` 添加 `vlm_prompt_path`、`vlm_model`、`vlm_use_ollama`、`ollama_host` 参数
- ✅ 添加 `_load_vlm_prompt_template()` 方法
- ✅ 添加 `_get_ollama_client()` 方法（懒加载 Ollama 客户端）
- ✅ 添加 `_analyze_environment_image()` 方法（支持两种模式）
- ✅ 添加 `_analyze_with_ollama()` 方法（本地 Ollama）
- ✅ 添加 `_analyze_with_openai_api()` 方法（远程 API）
- ✅ `plan_tasks()` 添加 `image_path` 参数
- ✅ 实现 VLM 调用逻辑（分析图片 → 生成环境理解 → 加入 prompt）

**新增方法**：
```python
def _get_ollama_client(self):
    """获取 Ollama 客户端（懒加载）"""

def _analyze_environment_image(self, image_path: str) -> Optional[str]:
    """使用 VLM 分析环境图像（支持 Ollama 和 API 两种模式）"""

def _analyze_with_ollama(self, image_path: str) -> Optional[str]:
    """使用本地 Ollama VLM 分析图像"""

def _analyze_with_openai_api(self, image_path: str) -> Optional[str]:
    """使用远程 OpenAI 兼容 API 分析图像"""
```

**VLM 调用逻辑**：
```python
# 1. 检查是否有 image_path
if image_path:
    # 2. 根据 vlm_use_ollama 选择模式
    vlm_result = self._analyze_environment_image(image_path)
    #    - True: 使用 Ollama（_analyze_with_ollama）
    #    - False: 使用 API（_analyze_with_openai_api）

    # 3. 将结果加入 prompt
    if vlm_result:
        user_input_section = f"【环境观察】\n{vlm_result}\n\n【用户指令】\n{user_input}"
```

**配置示例**：
```python
# 默认：本地 Ollama
llm = HighLevelLLM(
    api_key=api_key,
    vlm_use_ollama=True,          # 使用 Ollama
    vlm_model='qwen3-vl:4b',      # 模型名称
    ollama_host='http://localhost:11434'  # Ollama 地址
)

# 远程 API 模式
llm = HighLevelLLM(
    api_key=api_key,
    vlm_use_ollama=False,         # 使用远程 API
    vlm_model='qwen-vl-plus'      # API 模型
)
```

---

### ✅ 任务 1.3：修改 `llm_core.py`

**状态**：✅ 完成

**修改内容**：
- ✅ `LLMAgent.__init__` 自动查找 `vlm_perception.yaml`
- ✅ 传递 `vlm_prompt_path` 给 `HighLevelLLM`
- ✅ `run_pipeline()` 添加 `image_path` 参数
- ✅ `plan_tasks()` 添加 `image_path` 参数并传递给 HighLevelLLM

**关键修改**：
```python
# 获取VLM提示词路径
vlm_prompt_path = None
if prompt_path:
    prompts_dir = os.path.dirname(prompt_path)
    vlm_prompt_path = os.path.join(prompts_dir, "vlm_perception.yaml")

# 初始化 HighLevelLLM 时传递
self.high_level_llm = HighLevelLLM(
    api_key=api_key,
    base_url=base_url,
    prompt_path=prompt_path,
    vlm_prompt_path=vlm_prompt_path,  # VLM 提示词路径
    vlm_model="qwen-vl-plus"          # VLM 模型（可配置）
)

# run_pipeline 和 plan_tasks 都添加了 image_path 参数
```

---

### ✅ 任务 1.4：修改 `interactive.py`

**状态**：✅ 完成

**修改内容**：
- ✅ 添加图片路径提取逻辑（支持 3 种格式）
- ✅ 从用户输入中移除图片路径部分
- ✅ 传递 `image_path` 给 `run_pipeline()`

**支持的输入格式**：
```python
# 格式1: 明确指定图片
"根据 /path/to/image.png 前进"

# 格式2: 图片+指令
"/path/to/image.jpg 然后..."

# 格式3: 纯图片路径
"/path/to/image.jpeg"
```

**图片路径提取逻辑**：
```python
import re

# 匹配图片路径
image_patterns = [
    r'(?:根据|看看|观察|分析|检测)\s*([/\w\-./]+\.(?:png|jpg|jpeg))',
    r'([/\w\-./]+\.(?:png|jpg|jpeg))\s*(?:然后|并且|，|、)',
    r'^([/\w\-./]+\.(?:png|jpg|jpeg))$'
]

for pattern in image_patterns:
    match = re.search(pattern, user_input)
    if match:
        image_path = match.group(1)
        user_input = re.sub(pattern, '', user_input).strip()
        break
```

---

### 📊 第一阶段测试结果

**测试命令**：
```bash
python3 Interactive_Module/interactive.py
```

**测试用例**：

#### 测试1：带图片的指令 ✅
```bash
💬 根据 /home/xcj/work/FinalProject/VLM_Module/assets/green.png 前进

🖼️  [检测到图片] /home/xcj/work/FinalProject/VLM_Module/assets/green.png
🖼️  [VLM] 分析环境图像: ...
✅ [VLM] 环境理解完成：检测到绿色方块在前方...

🧠 [高层LLM] 任务规划中...
  步骤1: 向前移动1.0米
```

#### 测试2：纯图片路径 ✅
```bash
💬 /home/xcj/work/FinalProject/VLM_Module/assets/green.png

🖼️  [检测到图片] ...
✅ [VLM] 环境理解完成
🧠 [高层LLM] 任务规划中...
  （根据图片内容给出建议）
```

#### 测试3：普通指令（无图片）✅
```bash
💬 前进1米然后左转90度

（不调用VLM，正常工作）
🧠 [高层LLM] 任务规划中...
  步骤1: 前进1米
  步骤2: 左转90度
```

**结论**：✅ 第一阶段功能完全实现并测试通过！

---

## 🟠 第二阶段：执行监控逻辑实现 ✅ 已完成

**目标**：实现5种异常检测逻辑

**完成时间**：2025年2月2日

**状态**：✅ 完成

### 任务 2.1-2.5：实现5种异常检测 ✅ 已完成

**文件**：`/home/xcj/work/FinalProject/LLM_Module/execution_monitor.py`

**实现内容**：

1. **超时检测** (Task 2.1) ✅
   - 检测任务执行时间是否超过 `timeout_threshold`
   - 返回 `AnomalyType.TIMEOUT` 异常

2. **卡住检测** (Task 2.2) ✅
   - 检测机器人位置是否在 `stuck_threshold` 时间内不变
   - 使用 `_position_unchanged()` 方法判断位置是否相同
   - 返回 `AnomalyType.STUCK` 异常

3. **振荡检测** (Task 2.3) ✅
   - 记录最近20个位置点
   - 使用 `_detect_oscillation()` 方法检测振荡模式
   - 两种检测方法：位置偏移 + 方向变化
   - 返回 `AnomalyType.OSCILLATION` 异常

4. **传感器失效检测** (Task 2.4) ✅
   - 检查 `current_state["sensor_status"]` 字典
   - 识别状态为 "failed", "error", 或 False 的传感器
   - 返回 `AnomalyType.SENSOR_FAILURE` 异常

5. **环境变化检测** (Task 2.5) ✅
   - 支持两种检测方式：
     - 版本号：`current_state["environment_version"]` 变化
     - 标志位：`current_state["environment_changed"]` 为 True
   - 返回 `AnomalyType.ENVIRONMENT_CHANGE` 异常

---

### 任务 2.6：实现辅助方法 ✅ 已完成

**实现的方法**：

1. `_position_unchanged(pos1, pos2, threshold=0.01)` ✅
   - 检查两个位置是否相同
   - 使用欧几里得距离判断

2. `_calculate_distance(pos1, pos2)` ✅
   - 计算两点间欧几里得距离
   - 支持 3D 坐标 (x, y, z)

3. `_detect_oscillation(window_size=6)` ✅
   - 检测振荡行为（来回移动）
   - 两种检测方法：位置偏移 + 方向变化
   - 至少需要 6 个位置点

4. `_calculate_average_position(position_records)` ✅
   - 计算一组位置记录的平均位置
   - 用于位置分析

---

### 📊 第二阶段测试结果

**测试文件**：`LLM_Module/test_execution_monitor.py`

**测试结果**：
```
============================================================
测试结果汇总
============================================================
超时检测: ✅ 通过
卡住检测: ✅ 通过
振荡检测: ✅ 通过
传感器失效检测: ✅ 通过
环境变化检测（版本号）: ✅ 通过
环境变化检测（标志位）: ✅ 通过
辅助方法: ✅ 通过

============================================================
总计: 7 通过, 0 失败
============================================================

🎉 所有测试通过！
```

**结论**：✅ 第二阶段功能完全实现并通过测试！

---

## 🟢 第三阶段：后台监控启用 ✅ 已完成

**目标**：在任务执行时启动后台监控任务

**完成时间**：2025年2月2日

**状态**：✅ 完成

### 任务 3.1-3.4：实现后台监控功能 ✅ 已完成

**文件**：`/home/xcj/work/FinalProject/LLM_Module/adaptive_controller.py`

**实现内容**：

1. **启用 `_monitor_task_execution()` 后台监控** ✅
   - 实现后台监控循环
   - 定期检测异常（使用 `monitoring_interval`）
   - 正确处理 `asyncio.CancelledError`

2. **修改 `execute_with_monitoring()`** ✅
   - 启动后台监控任务（`asyncio.create_task()`）
   - 任务完成后取消监控
   - 检查监控到的异常并添加到结果中

3. **完善 `handle_execution_result()`** ✅
   - 处理监控检测到的异常
   - 触发重新规划
   - 根据异常类型选择重新规划级别

4. **启用 `_should_replan()`** ✅
   - 检查是否达到最大重试次数
   - 根据错误类型判断是否需要重新规划
   - 支持环境变化、障碍物、目标丢失等情况

5. **实现重新规划级别判断** ✅
   - `_determine_replan_level()` - 根据错误类型选择级别
   - `_determine_replan_level_from_anomaly()` - 根据异常类型选择级别

---

## 🧪 第四阶段：测试和验证 ✅ 已完成

**目标**：验证所有功能正常工作

**完成时间**：2025年2月2日

**状态**：✅ 完成

### 任务 4.1-4.3：全面测试 ✅ 已完成

**测试文件**：
1. `LLM_Module/test_execution_monitor.py` - 异常检测测试（7个测试，全部通过）
2. `LLM_Module/test_adaptive_controller.py` - 自适应控制流程测试（5个测试，全部通过）

**测试结果**：

#### 执行监控测试（7/7 通过）✅
```
超时检测: ✅ 通过
卡住检测: ✅ 通过
振荡检测: ✅ 通过
传感器失效检测: ✅ 通过
环境变化检测（版本号）: ✅ 通过
环境变化检测（标志位）: ✅ 通过
辅助方法: ✅ 通过

🎉 所有测试通过！
```

#### 自适应控制流程测试（5/5 通过）✅
```
场景1：正常执行: ✅ 通过
  - 规划次数: 1
  - 执行次数: 1
  - 重新规划次数: 0

场景2：卡住异常: ✅ 通过
  - 检测到卡住异常
  - 触发重新规划（PARAMETER_ADJUSTMENT）
  - 新任务成功执行

场景3：障碍物失败: ✅ 通过
  - 失败后自动重试3次
  - 第4次重试成功
  - 无需重新规划

场景4：多步骤任务: ✅ 通过
  - 正确执行2个子任务
  - 进度跟踪正常

场景5：超时异常: ✅ 通过
  - 检测到超时异常
  - 触发重新规划（PARAMETER_ADJUSTMENT）
  - 新任务成功执行

🎉 所有测试通过！
```

---

## 📝 执行清单

### 按顺序执行：

- [x] **第一阶段：VLM集成** ✅ 已完成
  - [x] 任务1.1: 创建 `vlm_perception.yaml`
  - [x] 任务1.2: 修改 `high_level_llm.py`
  - [x] 任务1.3: 修改 `llm_core.py`
  - [x] 任务1.4: 修改 `interactive.py`
  - [x] 测试 VLM 功能 ✅

- [x] **第二阶段：监控逻辑** ✅ 已完成
  - [x] 任务2.1: 实现超时检测 ✅
  - [x] 任务2.2: 实现卡住检测 ✅
  - [x] 任务2.3: 实现振荡检测 ✅
  - [x] 任务2.4: 实现传感器失效检测 ✅
  - [x] 任务2.5: 实现环境变化检测 ✅
  - [x] 任务2.6: 确认辅助方法存在 ✅
  - [x] 测试监控功能 ✅

- [x] **第三阶段：后台监控** ✅ 已完成
  - [x] 任务3.1: 启用 `_monitor_task_execution()` ✅
  - [x] 任务3.2: 修改 `execute_with_monitoring()` ✅
  - [x] 任务3.3: 完善 `handle_execution_result()` ✅
  - [x] 任务3.4: 启用 `_should_replan()` ✅
  - [x] 测试后台监控 ✅

- [x] **第四阶段：测试验证** ✅ 已完成
  - [x] 任务4.1: 测试 VLM 环境理解 ✅
  - [x] 任务4.2: 测试执行监控 ✅
  - [x] 任务4.3: 测试完整自适应流程 ✅

---

## 📝 执行清单

### 按顺序执行：

- [x] **第一阶段：VLM集成** ✅ 已完成
  - [x] 任务1.1: 创建 `vlm_perception.yaml`
  - [x] 任务1.2: 修改 `high_level_llm.py`
  - [x] 任务1.3: 修改 `llm_core.py`
  - [x] 任务1.4: 修改 `interactive.py`
  - [x] 测试 VLM 功能 ✅

- [x] **第二阶段：监控逻辑** ✅ 已完成
  - [x] 任务2.1: 实现超时检测 ✅
  - [x] 任务2.2: 实现卡住检测 ✅
  - [x] 任务2.3: 实现振荡检测 ✅
  - [x] 任务2.4: 实现传感器失效检测 ✅
  - [x] 任务2.5: 实现环境变化检测 ✅
  - [x] 任务2.6: 确认辅助方法存在 ✅
  - [x] 测试监控功能 ✅

- [ ] **第三阶段：后台监控**
  - [ ] 任务3.1: 启用 `_monitor_task_execution()`
  - [ ] 任务3.2: 修改 `execute_with_monitoring()`
  - [ ] 任务3.3: 完善 `handle_execution_result()`
  - [ ] 任务3.4: 启用 `_should_replan()`
  - [ ] 测试后台监控

- [ ] **第四阶段：测试验证**
  - [x] 任务4.1: 测试 VLM 环境理解 ✅
  - [x] 任务4.2: 测试执行监控 ✅
  - [ ] 任务4.3: 测试完整自适应流程

---

## ⚠️ 注意事项

### VLM 集成

1. **VLM 是可选的**
   - 没有图片时正常工作
   - VLM 失败时降级到纯文本模式

2. **性能考虑**
   - VLM 调用耗时（约1-3秒）
   - 可以考虑缓存结果

3. **错误处理**
   - VLM 模块未导入 → 跳过图像理解
   - 图片不存在 → 友好提示

### 监控逻辑

1. **状态来源**
   - 需要从 Sim_Module 获取机器人状态
   - 通过 ROS2 topic `/robot/state` 订阅

2. **阈值选择**
   - 超时：建议 30秒（可配置）
   - 卡住：建议 5秒（可配置）
   - 振荡：建议 6个位置点

3. **性能影响**
   - 监控频率不宜过高（建议0.1-0.5秒）
   - 避免频繁的重新规划

### 后台监控

1. **异步注意事项**
   - 使用 `asyncio.create_task()` 创建后台任务
   - 务必取消不再需要的任务
   - 捕获 `CancelledError`

2. **数据同步**
   - `env_state` 需要定期更新
   - 考虑使用回调或订阅机制

---

## 🎯 完成后的效果

### 基础场景：VLM 理解环境 ✅ 已完成

```bash
💬 根据 /path/to/obstacle.png 决定怎么走

🖼️  [检测到图片] /path/to/obstacle.png
🖼️  [VLM] 分析环境图像...
✅ [VLM] 环境理解完成：检测到红色方块在前方2米处，建议绕行

🧠 [高层LLM] 任务规划中...
  步骤1: 左转45度避让障碍物
  步骤2: 前进1.5米
  步骤3: 右转45度回到原方向
```

### 高级场景：自适应恢复（待完成）

```bash
💬 追击敌人

⚙️  [执行中] 追击最近的敌人
⚠️  [监控检测] 机器人卡住（5.2秒未移动）
🔄 [异常] 任务执行成功但检测到异常

🔄 [重新规划] 第 1 次 (级别: SKILL_REPLACEMENT)
✅ [重新规划] 策略: 后退解除卡住后继续

【新步骤 1/2】
✅ [成功] 后退0.5米解除卡住

【新步骤 2/2】
✅ [成功] 重新追击敌人

📊 [完成] 2/2 个任务成功
```

---

## 📊 当前进度

| 阶段 | 任务数 | 完成数 | 进度 |
|------|--------|--------|------|
| 第一阶段：VLM集成 | 4 | 4 | ✅ 100% |
| 第二阶段：监控逻辑 | 6 | 6 | ✅ 100% |
| 第三阶段：后台监控 | 4 | 4 | ✅ 100% |
| 第四阶段：测试验证 | 3 | 3 | ✅ 100% |
| **总计** | **17** | **17** | **✅ 100%** |

---

**🎉 全部完成！** ✅ LLM_Module 自适应控制功能已全部实现并通过全面测试！

**完成时间**：2025年2月2日
