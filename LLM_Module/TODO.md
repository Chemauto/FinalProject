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

## 🟠 第二阶段：执行监控逻辑实现

**目标**：实现5种异常检测逻辑

**预计时间**：2-3小时

### 任务 2.1：实现超时检测

**文件**：`/home/xcj/work/FinalProject/LLM_Module/execution_monitor.py`

**修改位置**：`detect_anomaly()` 方法（第102-141行）

**当前代码**：
```python
def detect_anomaly(self, current_state: Dict[str, Any], task: Dict[str, Any]) -> Optional[Anomaly]:
    """检测异常"""

    # ==================== 后续添加异常检测逻辑 ====================
    # TODO: 根据实际需求添加以下检测：
    # 1. 超时检测
    # 2. 卡住检测（位置不变）
    # 3. 振荡检测（来回移动）
    # 4. 传感器失效检测
    # 5. 环境变化检测
    # ===============================================================

    # 暂时不检测异常，返回None
    return None
```

**需要修改为**：
```python
def detect_anomaly(self, current_state: Dict[str, Any], task: Dict[str, Any]) -> Optional[Anomaly]:
    """检测异常"""

    current_time = time.time()

    # ==================== 1. 超时检测 ====================
    if self.execution_start_time:
        elapsed = current_time - self.execution_start_time
        if elapsed > self.timeout_threshold:
            return Anomaly(
                type=AnomalyType.TIMEOUT,
                description=f"任务执行超时（{elapsed:.1f}秒）",
                severity="high",
                data={"elapsed_time": elapsed, "threshold": self.timeout_threshold}
            )
    # ==========================================================

    # ==================== 2-5. 其他检测（见后续任务） ====================

    # 暂时不检测其他异常
    return None
```

**测试方法**：
```python
monitor = ExecutionMonitor(timeout_threshold=5.0)
monitor.execution_start_time = time.time()
time.sleep(6)  # 模拟超时
anomaly = monitor.detect_anomaly(current_state={}, task={})
# 应该返回超时异常
```

---

### 任务 2.2：实现卡住检测

**文件**：`/home/xcj/work/FinalProject/LLM_Module/execution_monitor.py`

**修改位置**：在超时检测后添加

**需要添加的代码**：
```python
    # ==================== 2. 卡住检测 ====================
    if current_state and "position" in current_state:
        current_position = current_state["position"]

        if self.last_position is not None:
            # 检查位置是否变化
            if self._position_unchanged(current_position, self.last_position):
                if self.last_position_update_time:
                    stuck_duration = current_time - self.last_position_update_time
                    if stuck_duration > self.stuck_threshold:
                        return Anomaly(
                            type=AnomalyType.STUCK,
                            description=f"机器人卡住（{stuck_duration:.1f}秒未移动）",
                            severity="medium",
                            data={
                                "stuck_duration": stuck_duration,
                                "position": current_position,
                                "threshold": self.stuck_threshold
                            }
                        )
            else:
                # 位置已更新，记录时间
                self.last_position_update_time = current_time

        # 保存当前位置
        self.last_position = current_position

        # 初始化位置更新时间
        if self.last_position_update_time is None:
            self.last_position_update_time = current_time
    # ==========================================================
```

**注意事项**：
- ✅ 依赖 `current_state["position"]` 包含 `{"x": ..., "y": ..., "z": ...}`
- ✅ 使用阈值判断位置是否不变（默认0.01米）
- ✅ 记录上次更新时间用于计算持续时间

---

### 任务 2.3：实现振荡检测

**文件**：`/home/xcj/work/FinalProject/LLM_Module/execution_monitor.py`

**修改位置**：在卡住检测后添加

**需要添加的代码**：
```python
                # 位置已更新，记录到历史
                self.last_position_update_time = current_time

                # ==================== 3. 振荡检测 ====================
                # 记录位置历史
                self.position_history.append({
                    "time": current_time,
                    "position": current_position
                })

                # 保持历史记录不超过20个
                if len(self.position_history) > 20:
                    self.position_history.pop(0)

                # 检测振荡（至少需要6个位置点）
                if len(self.position_history) >= 6:
                    if self._detect_oscillation():
                        return Anomaly(
                            type=AnomalyType.OSCILLATION,
                            description="检测到振荡行为（来回移动）",
                            severity="medium",
                            data={
                                "oscillation_count": len(self.position_history),
                                "positions": self.position_history[-6:]
                            }
                        )
                # ==========================================================
```

**注意事项**：
- ✅ 需要至少6个位置点才能检测振荡
- ✅ 限制历史记录长度避免内存占用
- ✅ 振荡定义：来回移动但最终回到原点附近

---

### 任务 2.4：实现传感器失效检测

**文件**：`/home/xcj/work/FinalProject/LLM_Module/execution_monitor.py`

**修改位置**：在振荡检测后添加

**需要添加的代码**：
```python
    # ==================== 4. 传感器失效检测 ====================
    if current_state and "sensor_status" in current_state:
        sensor_status = current_state["sensor_status"]

        # 检查各个传感器
        failed_sensors = []
        for sensor_name, status in sensor_status.items():
            if status == "failed" or status == "error" or status is False:
                failed_sensors.append(sensor_name)

        if failed_sensors:
            return Anomaly(
                type=AnomalyType.SENSOR_FAILURE,
                description=f"传感器失效: {', '.join(failed_sensors)}",
                severity="high",
                data={"failed_sensors": failed_sensors, "sensor_status": sensor_status}
            )
    # ==========================================================
```

**注意事项**：
- ✅ 检查 `sensor_status` 字典
- ✅ 支持多种失效状态：failed/error/False
- ✅ 返回所有失效的传感器列表

---

### 任务 2.5：实现环境变化检测

**文件**：`/home/xcj/work/FinalProject/LLM_Module/execution_monitor.py`

**修改位置**：在传感器检测后添加

**需要添加的代码**：
```python
    # ==================== 5. 环境变化检测 ====================
    if current_state and "environment_version" in current_state:
        # 使用版本号检测环境变化
        current_version = current_state["environment_version"]

        if hasattr(self, 'last_environment_version'):
            if current_version != self.last_environment_version:
                return Anomaly(
                    type=AnomalyType.ENVIRONMENT_CHANGE,
                    description="检测到环境变化",
                    severity="high",
                    data={
                        "old_version": self.last_environment_version,
                        "new_version": current_version
                    }
                )

        self.last_environment_version = current_version

    # 或者通过标志位检测
    if current_state and current_state.get("environment_changed", False):
        return Anomaly(
            type=AnomalyType.ENVIRONMENT_CHANGE,
            description="检测到环境变化",
            severity="high",
            data=current_state.get("environment_change_details", {})
        )
    # ==========================================================
```

**注意事项**：
- ✅ 两种检测方式：版本号或标志位
- ✅ 版本号方式更可靠（递增计数）
- ✅ 标志位方式简单直接

---

### 任务 2.6：保留辅助方法

**检查**：确保以下辅助方法存在于 `execution_monitor.py`

**需要的方法**：
```python
def _position_unchanged(self, pos1: Dict, pos2: Dict, threshold: float = 0.01) -> bool:
    """检查两个位置是否相同"""

def _detect_oscillation(self, window_size: int = 6) -> bool:
    """检测振荡行为"""

def _calculate_average_position(self, position_records: list) -> Dict:
    """计算平均位置"""

def _calculate_distance(self, pos1: Dict, pos2: Dict) -> float:
    """计算两点间欧几里得距离"""
```

**注意**：这些方法已经存在于原始代码中（第181-260行），只是被注释了。如果被注释，需要取消注释。

---

## 🟢 第三阶段：后台监控启用

**目标**：在任务执行时启动后台监控任务

**预计时间**：1小时

### 任务 3.1：启用 `_monitor_task_execution()` 后台监控

**文件**：`/home/xcj/work/FinalProject/LLM_Module/adaptive_controller.py`

**修改位置**：`_monitor_task_execution()` 方法（第184-194行）

**当前代码**：
```python
async def _monitor_task_execution(self, ...) -> Optional[Anomaly]:
    """监控任务执行（后台运行）- 暂未使用"""

    # TODO: 后续添加后台监控逻辑
    return None
```

**需要修改为**：
```python
async def _monitor_task_execution(self, ...) -> Optional[Anomaly]:
    """监控任务执行（后台运行）"""
    try:
        while True:
            # 定期检测异常
            anomaly = self.execution_monitor.detect_anomaly(
                current_state=env_state,
                task={"task": task.task, "type": task.type}
            )

            if anomaly:
                return anomaly

            # 等待下一次检查
            await asyncio.sleep(self.execution_monitor.monitoring_interval)

    except asyncio.CancelledError:
        # 任务被取消（正常结束）
        return None
```

---

### 任务 3.2：修改 `execute_with_monitoring()` 启动后台监控

**文件**：`/home/xcj/work/FinalProject/LLM_Module/adaptive_controller.py`

**修改位置**：`execute_with_monitoring()` 方法（第136-182行）

**当前代码**：
```python
async def execute_with_monitoring(self, ...):
    """带监控的任务执行（简化版）"""

    # TODO: 后续可以在这里添加：
    # 1. 后台监控任务 - 在执行时定期检测异常
    # ...

    # 重置监控器
    self.execution_monitor.reset()

    # 获取上一步结果
    previous_result = self._get_previous_result()

    try:
        # 直接执行任务（暂不启动后台监控）
        result = self.low_level_llm.execute_task(...)
        return result
```

**需要修改为**：
```python
async def execute_with_monitoring(self, ...):
    """带监控的任务执行"""

    # 重置监控器
    self.execution_monitor.reset()

    # 记录执行开始时间
    import time
    self.execution_monitor.execution_start_time = time.time()

    # 获取上一步结果
    previous_result = self._get_previous_result()

    # ==================== 启动后台监控任务 ====================
    monitoring_task = None
    if env_state:
        # 如果提供了环境状态，启动后台监控
        monitoring_task = asyncio.create_task(
            self._monitor_task_execution(task, env_state)
        )
    # ============================================================

    try:
        # 执行任务
        result = self.low_level_llm.execute_task(...)

        # ==================== 取消监控任务 ====================
        if monitoring_task and not monitoring_task.done():
            monitoring_task.cancel()
            try:
                await monitoring_task
            except asyncio.CancelledError:
                pass
        # ============================================================

        # ==================== 检查监控到的异常 ====================
        if monitoring_task and monitoring_task.done():
            anomaly = monitoring_task.result()
            if anomaly:
                print(f"⚠️  [监控检测] {anomaly.description}")
                if result.get("status") == "success":
                    result["anomaly_detected"] = True
                    result["anomaly"] = {
                        "type": anomaly.type.value,
                        "description": anomaly.description,
                        "severity": anomaly.severity
                    }
        # ============================================================

        return result
```

---

### 任务 3.3：完善 `handle_execution_result()` 的异常处理

**文件**：`/home/xcj/work/FinalProject/LLM_Module/adaptive_controller.py`

**修改位置**：`handle_execution_result()` 方法（第196-246行）

**当前代码**：
```python
async def handle_execution_result(self, ...):
    """处理执行结果（简化版）"""

    status = result.get("status", ExecutionStatus.FAILED.value)

    if status == ExecutionStatus.SUCCESS.value:
        # 任务成功
        print(f"✅ [成功] 任务完成: {task.task}")
        self.task_queue.mark_completed(task, result)

    # ...
```

**需要修改为**：
```python
async def handle_execution_result(self, ...):
    """处理执行结果"""

    status = result.get("status", ExecutionStatus.FAILED.value)

    if status == ExecutionStatus.SUCCESS.value:
        # ==================== 检查是否有异常 ====================
        if result.get("anomaly_detected"):
            # 监控器检测到异常，需要重新规划
            anomaly = result.get("anomaly", {})
            print(f"⚠️  [异常] 任务执行成功但检测到异常: {anomaly.get('description', 'Unknown')}")

            await self.trigger_replanning(
                task=task,
                result=result,
                env_state=env_state,
                available_skills=available_skills,
                level=self._determine_replan_level_from_anomaly(anomaly)
            )
        else:
            # ==================== 完全成功 ====================
            print(f"✅ [成功] 任务完成: {task.task}")
            self.task_queue.mark_completed(task, result)
        # ==========================================================
```

---

### 任务 3.4：启用 `_should_replan()` 的判断逻辑

**文件**：`/home/xcj/work/FinalProject/LLM_Module/adaptive_controller.py`

**修改位置**：`_should_replan()` 方法（第296-317行）

**当前代码**：
```python
def _should_replan(self, task: Task, result: Dict[str, Any]) -> bool:
    """判断是否应该重新规划（暂未使用）"""

    # TODO: 根据实际需求添加判断逻辑：
    # ...

    # 目前不自动重新规划
    return False
```

**需要修改为**：
```python
def _should_replan(self, task: Task, result: Dict[str, Any]) -> bool:
    """判断是否应该重新规划"""

    # 1. 如果达到最大重试次数，必须重新规划
    if not task.can_retry():
        return True

    # 2. 某些类型的错误需要重新规划
    error = result.get("error", "").lower()

    # 环境相关错误
    if "environment" in error:
        return True

    # 障碍物错误
    if "obstacle" in error or "blocked" in error:
        return True

    # 目标丢失
    if "target" in error and ("lost" in error or "not found" in error):
        return True

    # 其他情况不重新规划（使用重试机制）
    return False
```

---

## 🧪 第四阶段：测试和验证

**目标**：验证所有功能正常工作

### 任务 4.1：测试 VLM 环境理解 ✅ 已完成

**测试场景**：

#### 测试1：带图片的指令
```bash
💬 根据 /home/xcj/work/FinalProject/VLM_Module/assets/green.png 前进

预期输出：
🖼️  [检测到图片] /home/xcj/work/FinalProject/VLM_Module/assets/green.png
🖼️  [VLM] 分析环境图像: ...
✅ [VLM] 环境理解完成：检测到绿色方块在前方...
🧠 [高层LLM] 任务规划中...
```

#### 测试2：纯图片路径
```bash
💬 /home/xcj/work/FinalProject/VLM_Module/assets/green.png

预期输出：
（VLM 分析图片内容并给出建议）
```

#### 测试3：普通指令（确保向后兼容）
```bash
💬 前进1米然后左转90度

预期输出：
（不调用VLM，正常工作）
```

---

### 任务 4.2：测试执行监控

**测试步骤**：

1. 修改监控阈值（便于测试）
   ```python
   # 在 interactive.py 中临时修改
   monitor = ExecutionMonitor(
       monitoring_interval=1.0,      # 1秒检查一次
       timeout_threshold=5.0,       # 5秒超时
       stuck_threshold=3.0          # 3秒卡住
   )
   ```

2. 测试超时检测
   ```
   # 输入一个会卡住的任务
   前进100米  # 假设会导致超时
   ```

3. 预期输出
   ```
   ⚠️  [监控检测] 任务执行超时（5.2秒）
   🔄 [异常] 任务执行成功但检测到异常: ...
   🔄 [重新规划] 第 1 次 (级别: PARAMETER_ADJUSTMENT)
   ```

---

### 任务 4.3：测试完整自适应流程

**测试场景**：

#### 场景1：任务卡住 → 自动重新规划
```
1. 机器人前进
2. 人为设置障碍物（在Sim_Module中）
3. 观察是否检测到"卡住"
4. 观察是否重新规划（后退→再前进）
```

#### 场景2：环境变化 → 完全重新规划
```
1. 追击敌人
2. 中途删除敌人（Sim_Module中按C清除）
3. 观察是否检测到"环境变化"
4. 观察是否重新规划（搜索→追击）
```

#### 场景3：VLM理解 + 自适应
```
1. 提供包含障碍物的图片
2. 输入"前进到目标"
3. 观察VLM是否识别障碍物
4. 观察任务是否避开障碍物
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

- [ ] **第二阶段：监控逻辑**
  - [ ] 任务2.1: 实现超时检测
  - [ ] 任务2.2: 实现卡住检测
  - [ ] 任务2.3: 实现振荡检测
  - [ ] 任务2.4: 实现传感器失效检测
  - [ ] 任务2.5: 实现环境变化检测
  - [ ] 任务2.6: 确认辅助方法存在
  - [ ] 测试监控功能

- [ ] **第三阶段：后台监控**
  - [ ] 任务3.1: 启用 `_monitor_task_execution()`
  - [ ] 任务3.2: 修改 `execute_with_monitoring()`
  - [ ] 任务3.3: 完善 `handle_execution_result()`
  - [ ] 任务3.4: 启用 `_should_replan()`
  - [ ] 测试后台监控

- [ ] **第四阶段：测试验证**
  - [x] 任务4.1: 测试 VLM 环境理解 ✅
  - [ ] 任务4.2: 测试执行监控
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
| 第二阶段：监控逻辑 | 6 | 0 | 0% |
| 第三阶段：后台监控 | 4 | 0 | 0% |
| 第四阶段：测试验证 | 3 | 1 | 33% |
| **总计** | **17** | **5** | **29%** |

---

**✅ 第一阶段完成！** 🎉 VLM 环境理解功能已成功集成并通过测试！

**下一步：继续第二阶段（执行监控逻辑实现）**
